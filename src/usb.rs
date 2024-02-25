use crate::{commands::CommandContext, hal};
use anchor::*;
use core::mem::MaybeUninit;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
pub use hal::otg_fs::{UsbBusType as UsbBus, USB};
use usb_device::{
    bus,
    device::{self, StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    LangID, UsbError,
};
use usbd_serial::CdcAcmClass;

pub const USB_OUTPUT_BUFFER_SIZE: usize = 256;
pub const USB_INPUT_BUFFER_SIZE: usize = 256;
pub const USB_MAX_PACKET_SIZE: usize = 64;

pub type UsbBusAllocator = bus::UsbBusAllocator<UsbBus>;

pub struct UsbDevice {
    device: device::UsbDevice<'static, UsbBus>,
    serial: CdcAcmClass<'static, UsbBus>,
    rx_buffer: FifoBuffer<USB_INPUT_BUFFER_SIZE>,
    tx_consumer: bbqueue::Consumer<'static, USB_OUTPUT_BUFFER_SIZE>,
}

impl UsbDevice {
    pub fn init(
        allocator: &'static mut UsbBusAllocator,
        tx_queue: &'static bbqueue::BBBuffer<USB_OUTPUT_BUFFER_SIZE>,
    ) -> Self {
        let serial = CdcAcmClass::new(allocator, USB_MAX_PACKET_SIZE as u16);
        let strings = StringDescriptors::new(LangID::EN)
            .product("Flycron")
            .manufacturer("Armchair Heavy Industries")
            .serial_number(crate::chipid::get_chipid());
        let device = UsbDeviceBuilder::new(allocator, UsbVidPid(0x1d50, 0x614e))
            .composite_with_iads()
            .strings(&[strings])
            .unwrap()
            .build();

        let (tx_producer, tx_consumer) = tx_queue.try_split().unwrap();
        unsafe {
            USB_TX_PRODUCER = MaybeUninit::new(tx_producer);
        }

        Self {
            device,
            serial,
            rx_buffer: FifoBuffer::new(),
            tx_consumer,
        }
    }

    pub fn on_interrupt(&mut self) -> bool {
        if self.device.poll(&mut [&mut self.serial]) {
            self.read_all();
        }
        !self.rx_buffer.is_empty()
    }

    pub fn read_all(&mut self) {
        loop {
            let target = self.rx_buffer.receive_buffer();
            if target.len() < USB_MAX_PACKET_SIZE {
                break;
            }
            match self.serial.read_packet(target) {
                Ok(n) => self.rx_buffer.advance(n),
                _ => break,
            }
        }
    }

    pub fn handle_commands(&mut self, context: CommandContext) {
        if self.rx_buffer.is_empty() {
            return;
        }
        let mut wrap = SliceInputBuffer::new(self.rx_buffer.data());
        crate::KLIPPER_TRANSPORT.receive(&mut wrap, context);
        self.rx_buffer.pop(self.rx_buffer.len() - wrap.available());
    }

    pub fn pump_write(&mut self) {
        loop {
            let grant = match self.tx_consumer.read() {
                Ok(grant) => grant,
                Err(bbqueue::Error::InsufficientSize) => return,
                Err(e) => {
                    defmt::error!("tx_consumer read error: {}", e);
                    return;
                }
            };
            let len = grant.buf().len();
            let take = len.clamp(0, self.serial.max_packet_size() as usize);

            match self.serial.write_packet(&grant.buf()[..take]) {
                Ok(n) => grant.release(n),
                Err(UsbError::WouldBlock) => {
                    USB_TX_WAITING.signal(());
                    return;
                }
                Err(e) => {
                    grant.release(take);
                    defmt::error!("USB send error: {}", e);
                }
            }
        }
    }

    pub async fn tx_wait() {
        USB_TX_WAITING.wait().await
    }
}

static mut USB_TX_PRODUCER: MaybeUninit<bbqueue::Producer<'static, USB_OUTPUT_BUFFER_SIZE>> =
    MaybeUninit::uninit();
static USB_TX_WAITING: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub struct BufferTransportOutput;
pub const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

impl anchor::TransportOutput for BufferTransportOutput {
    type Output = anchor::ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = anchor::ScratchOutput::new();
        f(&mut scratch);
        let output = scratch.result();

        let producer = unsafe { USB_TX_PRODUCER.assume_init_mut() };
        if let Ok(mut grant) = producer.grant_exact(output.len()) {
            grant.buf().copy_from_slice(output);
            grant.commit(output.len());
            USB_TX_WAITING.signal(());
        }
    }
}
