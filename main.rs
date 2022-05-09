#[derive(Debug, Copy, Clone)]
struct Vec3 {
    x: f64, 
    y: f64, 
    z: f64
}

impl Vec3 {
    fn sub(&self, other: Vec3) -> Vec3 {
        return Vec3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        };
    }

    // fn add(&self, other: Vec3) -> Vec3 {
    //     return Vec3 {
    //         x: self.x + other.x,
    //         y: self.y + other.y,
    //         z: self.z + other.z,
    //     };
    // }

    fn cross(&self, other: Vec3) -> Vec3 {
        return Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        };
    }

    fn dot(&self, other: Vec3) -> f64 {
        return self.x * other.x + self.y * other.y + self.z * other.z;
    }

    fn lerp(&self, other: Vec3, t: f64) -> Vec3 {
        return Vec3 {
            x: (1. - t) * self.x + t * other.x,
            y: (1. - t) * self.y + t * other.y,
            z: (1. - t) * self.z + t * other.z,
        };
    }

    fn scale(&self, scale: f64) -> Vec3 {
        return Vec3 {
            x: self.x * scale,
            y: self.y * scale,
            z: self.z * scale,
        };
    }

    fn normalized(&self) -> Vec3 {
        let len = self.len();
        return self.scale(1. / len);
    }

    fn len(&self) -> f64 {
        return (self.dot(*self)).sqrt();
    }

    // fn dist(&self, other: Vec3) -> f64 {
    //     return self.sub(other).len();
    // }
}

#[derive(Debug, Copy, Clone)]
struct BSPPlane {
    n: Vec3,
    d: f64
}

#[allow(dead_code)]
struct InnerBSPNode {
    plane: BSPPlane,
    front: Box<BSPNode>,
    back: Box<BSPNode>,
    polygons: Vec<BSPPolygon>
}

enum BSPNode {
    Node(InnerBSPNode),
    Leaf
}   

#[derive(Clone)]
struct BSPPolygon {
    plane: BSPPlane,
    vertices: Vec<Vec3>
}

enum PolygonPlaneSide {
    FRONT,
    BACK,
    SPANNING,
    COPLANAR
}

fn classify_polygon_by_plane(plane: BSPPlane, polygon: &BSPPolygon) -> PolygonPlaneSide {
    let mut front_count = 0;
    let mut back_count = 0;

    for p in polygon.vertices.iter() {
        match classify_point_to_plane(plane, *p) {
            PointPlaneSide::FRONT => {
                front_count = front_count + 1;
            }
            PointPlaneSide::BACK => {
                back_count = back_count + 1;
            }
            _ => {}
        }

        if front_count != 0 && back_count != 0 {
            return PolygonPlaneSide::SPANNING;
        }
    }

    if front_count != 0 {
        return PolygonPlaneSide::FRONT;
    }

    if back_count != 0 {
        return PolygonPlaneSide::BACK;
    }

    return PolygonPlaneSide::COPLANAR;
}

fn bsp_plane_by_three_points(a: Vec3, b: Vec3, c: Vec3) -> BSPPlane {
    let d0 = b.sub(a);
    let d1 = c.sub(a);

    let n = d1.cross(d0).normalized();
    let d = a.dot(n);

    return BSPPlane { n,  d };
}

fn bsp_polygon_by_vertices(vertices: Vec<Vec3>) -> BSPPolygon {
    return BSPPolygon {
        plane: bsp_plane_by_three_points(vertices[0], vertices[1], vertices[2]),
        vertices: vertices
    }
}

fn intersect_segment_plane(a: Vec3, b: Vec3, plane: BSPPlane) -> Option<Vec3> {
    let ab = b.sub(a);
    let t = (plane.d - plane.n.dot(a)) / plane.n.dot(ab);

    if t >= 0.0 && t <= 1.0 {
        return Some(a.lerp(b, t));
    }

    return None;
}

enum PointPlaneSide {
    COPLANAR,
    FRONT,
    BACK,
}

impl PartialEq for PointPlaneSide {
    fn eq(&self, other: &Self) -> bool {
        self == other
    }
}

const PLANE_THICKNESS_EPS: f64 = 1e-6;

fn classify_point_to_plane(plane: BSPPlane, p: Vec3) -> PointPlaneSide {
    let dist = plane.n.dot(p) - plane.d;

    if dist > PLANE_THICKNESS_EPS {
        return PointPlaneSide::FRONT;
    } else if dist < -PLANE_THICKNESS_EPS {
        return PointPlaneSide::BACK;
    }

    return PointPlaneSide::COPLANAR;
}

fn split_bsp_polygon(splitting_plane: BSPPlane, polygon: &BSPPolygon) -> (BSPPolygon, BSPPolygon) {
    let plane = splitting_plane;
    let BSPPolygon { vertices: points, .. } = polygon;

    let mut a = points[points.len() - 1];
    let mut a_side = classify_point_to_plane(plane, a);

    let mut front_verts: Vec<Vec3> = Vec::new();
    let mut back_verts: Vec<Vec3> = Vec::new();

    for br in points.iter() {
        let b = *br;
        let b_side = classify_point_to_plane(plane, b);

        if b_side == PointPlaneSide::FRONT {
            if a_side == PointPlaneSide::BACK {
                let int = intersect_segment_plane(b, a, plane);

                if int.is_some() {
                    front_verts.push(int.unwrap());
                    back_verts.push(int.unwrap());
                }
            }

            front_verts.push(b);
        } else if b_side == PointPlaneSide::BACK {
            if a_side == PointPlaneSide::FRONT {
                let int = intersect_segment_plane(a, b, plane);

                if int.is_some() {
                    front_verts.push(int.unwrap());
                    back_verts.push(int.unwrap());
                }
            } else if a_side == PointPlaneSide::COPLANAR {
                back_verts.push(a);
            }

            back_verts.push(b);
        } else {
            front_verts.push(b);

            if a_side == PointPlaneSide::BACK {
                back_verts.push(b);
            }
        }

        a = b;
        a_side = b_side;
    }

    return ( BSPPolygon {
        plane: polygon.plane,
        vertices: front_verts
    }, BSPPolygon {
        plane: polygon.plane,
        vertices: back_verts
    });
}

fn build_bsp_node(polygons: Vec<BSPPolygon>) -> BSPNode {
    if polygons.len() == 0 {
        return BSPNode::Leaf
    }

    let mut front: Vec<BSPPolygon> = Vec::new();
    let mut back: Vec<BSPPolygon> = Vec::new();
    let mut coplanar: Vec<BSPPolygon> = Vec::new();

    let split_plane: BSPPlane = polygons[0].plane;

    for polygon in polygons.iter() {
        let class = classify_polygon_by_plane(split_plane, polygon);

        match class {
            PolygonPlaneSide::BACK => back.push(polygon.clone()),
            PolygonPlaneSide::FRONT => front.push(polygon.clone()),
            PolygonPlaneSide::COPLANAR => coplanar.push(polygon.clone()),
            PolygonPlaneSide::SPANNING => {
                let (front_poly, back_poly) = split_bsp_polygon( split_plane, polygon );
                back.push(back_poly);
                front.push(front_poly);
            },
        }
    }

    BSPNode::Node(InnerBSPNode {
        plane: split_plane,
        front: Box::new(build_bsp_node(front)),
        back: Box::new(build_bsp_node(back)),
        polygons: coplanar
    })
}

fn bsp_cube_faces(center: Vec3, radius: Vec3) -> Vec<BSPPolygon> {
    let verts = vec!(
        ([0, 4, 6, 2], [-1, 0, 0]),
        ([1, 3, 7, 5], [1, 0, 0]),
        ([0, 1, 5, 4], [0, -1, 0]),
        ([2, 6, 7, 3], [0, 1, 0]),
        ([0, 2, 3, 1], [0, 0, -1]),
        ([4, 5, 7, 6], [0, 0, 1])
    );

    return verts.iter().map(|v| {
        return bsp_polygon_by_vertices(v.0.iter().map(|i| {
            return Vec3 {
                x: center.x + radius.x * (2. * if i & 1 != 0 { 1. } else { 0. } - 1.),
                y: center.y + radius.y * (2. * if i & 2 != 0 { 1. } else { 0. } - 1.),
                z: center.z + radius.z * (2. * if i & 4 != 0 { 1. } else { 0. } - 1.),
            };
        }).collect());
    }).collect();
}

fn render_bsp(node: &BSPNode, viewer: Vec3) {
    match node {
        BSPNode::Node(node) => {
            println!("node!");

            if node.plane.n.dot(viewer) > node.plane.d {
                render_bsp(&*node.front, viewer);
                render_bsp(&*node.back, viewer);
            } else {
                render_bsp(&*node.back, viewer);
                render_bsp(&*node.front, viewer);
            }
        }
        BSPNode::Leaf => {
            println!("leaf!");
        }
    }
}

fn main() {
    let node = build_bsp_node(bsp_cube_faces(Vec3{x: 0.,y: 0.,z: 0.}, Vec3{x: 5.,y: 5.,z: 5.}));
    render_bsp(&node, Vec3{x: 10.,y: 10.,z: 0.});
}


