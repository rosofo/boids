use glium as gl;
use nalgebra_glm as na;

#[derive(Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 2],
}

gl::implement_vertex!(Vertex, position);

pub fn vec2_to_vertex(v: &na::Vec2) -> Vertex {
    Vertex { position: *v.as_ref() }
}

pub fn solid_color_shader(r: f32, g: f32, b: f32, a: f32) -> String {
    format!(r#"
        #version 140

        out vec4 color;

        void main() {{
            color = vec4({}, {}, {}, {});
        }}
    "#, r, g, b, a)
}

pub fn simple_program(display: &gl::backend::glutin::Display)
    -> Result<gl::Program, gl::ProgramCreationError> {
    let vertex_shader_src = r#"
        #version 140

        in vec2 position;

        void main() {
        gl_Position = vec4(position, 0.0, 1.0);
        }
    "#;

    gl::Program::from_source(display, vertex_shader_src,
                             &solid_color_shader(0.0, 0.05, 0.2, 1.0), None)
}

pub fn mean(vectors: &Vec<na::Vec2>) -> na::Vec2 {
    let sum: na::Vec2 = vectors.iter().sum();
    sum / (vectors.len() as f32)
}

pub mod shapes {
    use nalgebra_glm as na;

    pub struct Shape {
        pub vertices: Vec<na::Vec2>,
        pub indices:  Vec<u32>,
    }

    pub fn boid(scale: f32) -> Shape {
        let m = na::mat3_to_mat2(&na::scale2d(&na::identity(), &na::vec2(scale, scale)));
        Shape {
            vertices: vec![
                na::vec2(0.0, 1.0),
                na::vec2(0.0, 0.0),
                na::vec2(-1.0, -1.0),
                na::vec2(1.0, -1.0),
            ].iter().map(|v| m * v).collect(),
            indices: vec![0, 1, 2, 0, 1, 3],
        }
    } 
}