mod aabb;

use aabb::AABB;
use math::{Point3, Vec3};

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

impl<T> Octree<T> {
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
        // if point is outside of the current octree's bounding box, then expand the octree
    }
    pub fn nearest_neighbor(&self, pos: Point3) -> (Point3, T) {}
    pub fn search(&self, pos: Point3, radius: f32) -> impl Iterator<Item = (Point3, T)> {}
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
