# Flycron controller

```console
% % DEFMT_LOG=info cargo run --release
   Compiling flycron v0.1.0 (/home/dalegaard/annex/flycron)
    Finished release [optimized + debuginfo] target(s) in 1.93s
     Running `probe-rs run --chip STM32F411CEUx thumbv7em-none-eabihf/release/flycron`
      Erasing ✔ [00:00:00] [####################################################################] 32.00 KiB/32.00 KiB @ 44.95 KiB/s (eta 0s )
  Programming ✔ [00:00:00] [####################################################################] 31.00 KiB/31.00 KiB @ 33.58 KiB/s (eta 0s )    Finished in 1.658s
0 INFO  init
└─ flycron::app::init @ /home/dalegaard/annex/flycron/src/main.rs:36
```
