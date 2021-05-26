use array_init;

mod aabb;
mod intersection;

use aabb::AABB;
use math::{Point3, Vec3};
use intersection::*;

pub trait Policy {
    const POLICY: PolicyEnum;
}

pub enum PolicyEnum {
    MaxElements(usize),
}

pub struct MaxElements<const N: usize>;
impl<const N: usize> Policy for MaxElements<N> {
    const POLICY: PolicyEnum = PolicyEnum::MaxElements(N);
}

#[derive(Debug, Clone)]
pub enum Node<T> {
    Leaf {
        position: Point3,
        value: T,
    },
    Branch {
        children: Box<[Node<T>; 8]>,
        split: Point3,
    },
    Empty,
}

pub struct Octree<T> {
    pub root: Node<T>,
    pub aabb: AABB,
}

impl<T: Default + Copy> Octree<T> {
    pub fn new() -> Self {
        Octree {
            root: Node::Empty,
            aabb: AABB::empty(),
        }
    }

    pub fn insert<F>(&mut self, pos: Point3, val: T, needs_split: F)
    where
        F: Fn(Node<T>) -> bool,
    {
        if self.aabb.contains(pos) {
            let mut node_ref = &mut self.root;
            // descend and insert
            loop {
                match node_ref {
                    Node::Branch { children, split } => {
                        // do zyx split
                        // 000 == x < split.x, y < split.y, z < split.z
                        // etc.
                        let index: usize = ((pos.z() < split.z()) as usize)
                            << 2 + ((pos.y() < split.y()) as usize)
                            << 1 + (pos.x() < split.x()) as usize;
                        // overwrite ref, not contents of ref. notice no * before node_ref
                        node_ref = &mut children[index];
                    }
                    Node::Leaf { position, value } => {
                        // node was a leaf, now needs to be a branch.
                        let split = (Vec3::from(*position) + Vec3::from(pos)) / 2.0;
                        let index1: usize = ((pos.z() < split.z()) as usize)
                            << 2 + ((pos.y() < split.y()) as usize)
                            << 1 + (pos.x() < split.x()) as usize;
                        let index2: usize = ((position.z() < split.z()) as usize)
                            << 2 + ((position.y() < split.y()) as usize)
                            << 1 + (position.x() < split.x()) as usize;

                        let mut children: Box<[Node<T>; 8]> =
                            Box::new(array_init::array_init(|i| Node::Empty));

                        children[index1] = Node::Leaf {
                            position: pos,
                            value: val,
                        };
                        children[index2] = Node::Leaf {
                            position: *position,
                            value: *value,
                        };

                        *node_ref = Node::Branch {
                            children,
                            split: Point3::from(split),
                        };
                        // insertion completed
                        return;
                    }
                    Node::Empty => {
                        *node_ref = Node::Leaf {
                            position: pos,
                            value: val,
                        };
                        // insertion completed
                        return;
                    }
                }
            }
        } else {
            // expand

            // need to choose where the central split is when growing. based on where the new point is.
            // if the new point is already between min and max for any dimension, there is ambiguity and the octree could expand along that dimension in any direction.

            self.aabb = AABB::grow(self.aabb, &pos);

            match self.root {
                Node::Empty => {
                    self.root = Node::Leaf {
                        position: pos,
                        value: val,
                    };
                }
                Node::Leaf { .. } => {}
                Node::Branch { .. } => {}
            }
        }
    }
    pub fn nearest_neighbor(&self, pos: Point3) -> (Point3, T) {
        (Point3::ZERO, T::default())
    }
    pub fn search(&self, pos: Point3, radius: f32) -> Vec<(Point3, T)> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
