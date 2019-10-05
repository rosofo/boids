use glium as gl;

#[derive(Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 2],
}

gl::implement_vertex!(Vertex, position);

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
                             &solid_color_shader(1.0, 0.0, 0.0, 1.0), None)
}