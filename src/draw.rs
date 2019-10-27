use crate::geometry::Model;
use gl::Surface;
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

pub struct Drawer {
    pub display: gl::Display,
    pub program: gl::Program,
    vertices: Vec<Vec<Vertex>>,
    types: Vec<gl::index::PrimitiveType>,
}

pub fn drawer(display: gl::Display, program: gl::Program) -> Drawer {
    Drawer {
        display: display,
        program: program,
        vertices: Vec::new(),
        types: Vec::new(),
    }
}

impl Drawer {
    pub fn add_model(&mut self, Model(verts, kind): Model) {
        let indices_types = self.types.iter().enumerate();
        let vertices = &mut self.vertices;

        indices_types.for_each(|(i, prim_type)| {
            let converted = verts.iter().map(vec3_to_vertex);
            if *prim_type == kind {
                vertices[i].extend(converted);
            }
        });
    }

    pub fn draw(&mut self, frame: &mut gl::Frame) {
        let types_vertices = self.types.iter().zip(self.vertices.iter());

        types_vertices.for_each(|(prim_type, verts)| {
            self.draw_vertices(frame, verts, *prim_type);
        });
    }

    pub fn draw_vertices(
        &self,
        frame: &mut gl::Frame,
        verts: &[Vertex],
        primitive_type: gl::index::PrimitiveType,
    ) {
        let vertex_buffer = gl::vertex::VertexBuffer::new(&self.display, verts).unwrap();

        frame.clear_color(0.0, 0.0, 0.0, 1.0);
        frame
            .draw(
                &vertex_buffer,
                &gl::index::NoIndices(primitive_type),
                &self.program,
                &gl::uniform! {},
                &Default::default(),
            )
            .unwrap();
    }
}
