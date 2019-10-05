use glium as gl;

#[derive(Copy, Clone)]
pub struct Vertex {
    position: [f32; 3],
    texcoords: [f32; 2],
}

gl::implement_vertex!(Vertex, position, texcoords);