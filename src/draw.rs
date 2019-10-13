use glium as gl;
use nalgebra_glm as na;

#[derive(Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 3],
}

gl::implement_vertex!(Vertex, position);

pub fn vec3_to_vertex(v: &na::Vec3) -> Vertex {
    Vertex {
        position: v.xyz().into(),
    }
}

pub fn solid_color_shader(r: f32, g: f32, b: f32, a: f32) -> String {
    format!(
        r#"
        #version 140

        out vec4 color;

        void main() {{
            color = vec4({}, {}, {}, {});
        }}
    "#,
        r, g, b, a
    )
}

pub fn simple_program(
    display: &gl::backend::glutin::Display,
) -> Result<gl::Program, gl::ProgramCreationError> {
    let vertex_shader_src = r#"
        #version 140

        in vec3 position;

        void main() {
        gl_Position = vec4(position, 1.0);
        }
    "#;

    gl::Program::from_source(
        display,
        vertex_shader_src,
        &solid_color_shader(0.0, 0.05, 0.2, 1.0),
        None,
    )
}
