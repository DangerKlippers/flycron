use crate::stepper_emulation::StepperMove;

#[derive(PartialEq)]
#[derive(Copy, Clone)]
pub struct MoveNode {
    //next: MoveNode,
}

impl MoveNode {
    pub fn setup() -> MoveNode {
        MoveNode {}
    }
}

pub struct MoveQueue {
    first: Option<MoveNode>,
    last: Option<MoveNode>,
}

impl MoveQueue {
    pub fn setup() -> MoveQueue {
        MoveQueue {
            first: None,
            last: None,
        }
    }

    pub fn clear(&mut self) {
        self.first = Some(MoveNode {});
    }

    pub fn is_empty(&mut self) -> bool {
        return self.first == None;
    }

    pub fn push(&mut self, _move_node: MoveNode) -> u8 {
        let mut move_node = _move_node;
        // move_node.next = None;

        if self.first.is_some() {
            //self.last.next = move_node;
            self.last = Some(move_node);
            return 0;
        }
        self.first = Some(move_node);
        self.last = Some(move_node);
        return 1;
    }
}
