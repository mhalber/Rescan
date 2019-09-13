/*
  ==============================================================================
  
  DEBUGVIS.H - WIP! 
  
  This header file provides debug rendering functionality:
  1. Thick line rendering
  2. Grid rendering
  3. Axis rendering

  #define DEBUG_IMPLEMENTATION
  #include "debug.h"

  ==============================================================================
  DEPENDENCIES
    msh.h
    msh_vec_math.h
    msh_gfx.h
    msh_geometry.h
    
  ==============================================================================
  AUTHORS

    Maciej S. Halber (macikuh@gmail.com)
  ==============================================================================
  LICENSE

  Copyright (c) 2017 Maciej S. Halber

  Permission is hereby granted, free of charge, to any person obtaining a copy 
  of this software and associated documentation files (the "Software"), to deal 
  in the Software without restriction, including without limitation the rights 
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
  copies of the Software, and to permit persons to whom the Software is 
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
  SOFTWARE.

  ==============================================================================
  NOTES:
  This should be more like rendering api - user requests some stuff, like
  grid or axes of specific sizes, and those are generated on the fly and rendered.
  This is debug api, so it does not have to be the fastests.
  ==============================================================================
  TODOs:
  [x] Remove multi-step deffered stuff.
  ==============================================================================
  REFERENCES:
    
*/
#ifndef DEBUGVIS_H
#define DEBUGVIS_H

typedef enum dbgvis_shader_mode
{
  DBGVIS_LINE       = 0,
  DBGVIS_THICK_LINE = 1,
  DBGVIS_N_SHDR_MODES,
} dbgvis_shader_mode_t;

typedef enum dbgvis_color_mode
{
  DBGVIS_FLAT_COLOR = 0,
  DBGVIS_N_CLR_MODES
} dbgvis_color_mode_t;

typedef struct dbgvis_display_options
{
  msh_vec4_t viewport;
  dbgvis_shader_mode_t shader_mode;
  dbgvis_color_mode_t  color_mode;
  float line_width;
  int show_axis;
  int show_grid;
  float axis_scale;
  float grid_scale;
} dbgvis_display_opts_t;

typedef struct dbgvis_ogl_state
{
  GLint active_texture;
  GLint active_program;
  GLint array_buffer;
  GLint vertex_array;
  GLint blend_src_rgb;
  GLint blend_dst_rgb;
  GLint blend_src_alpha;
  GLint blend_dst_alpha;
  GLint blend_equation_rgb;
  GLint blend_equation_alpha;
  GLboolean enable_blend;
  GLboolean enable_cull_face;
  GLboolean enable_depth_test;
  GLboolean enable_scissor_test;
} dbgvis_ogl_state_t;

typedef struct dbgvis_shader_source
{
  char *flat_vs, *flat_fs, *line_gs;
} dbgvis_shader_source_t;

typedef struct dbgvis_ctx
{
  mshgfx_geometry_t screen_geo;
  mshgfx_geometry_t axes_geo; 
  mshgfx_geometry_t grid_geo; 

  mshgfx_shader_prog_t line_shdrs[2];

  dbgvis_display_opts_t display_opts;
  dbgvis_ogl_state_t ogl_state_backup;
  dbgvis_shader_source_t shader_source;
  msh_camera_t* camera;
} dbgvis_ctx_t;


void dbgvis_render_axis(dbgvis_ctx_t* ctx, msh_mat4_t pose);
void dbgvis_render_grid(dbgvis_ctx_t* ctx);

void dbgvis_new_frame(dbgvis_ctx_t* ctx, msh_camera_t* cam);
void dbgvis_end_frame(dbgvis_ctx_t *ctx);

int dbgvis_init_context(dbgvis_ctx_t* ctx);
int dbgvis_destroy_context(dbgvis_ctx_t* ctx);

void dbgvis_keyboard_shortcuts( dbgvis_ctx_t* ctx, int* pressed, bool ctrl_down );
#ifdef DEBUGVIS_USE_NK_GUI
void dbgvis_nk_gui(dbgvis_ctx_t* dbgvis_ctx, struct nk_context *nk_ctx);
#endif

#endif /*DEBUGVIS_H*/


#ifdef DEBUGVIS_IMPLEMENTATION


////////////////////////////////////////////////////////////////////////////////
// DEBUGVIS Rendering
////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUGVIS_USE_NK_GUI
void 
dbgvis_nk_gui(dbgvis_ctx_t *dbgvis_ctx, struct nk_context *nk_ctx)
{
  dbgvis_display_opts_t* opts = &dbgvis_ctx->display_opts; 
  nk_layout_row_dynamic(nk_ctx, 20, 1);
  nk_checkbox_label(nk_ctx, "Show Axis", &opts->show_axis);
  nk_checkbox_label(nk_ctx, "Show Grid", &opts->show_grid);
}
#endif

void 
dbgvis_keyboard_shortcuts( dbgvis_ctx_t* ctx, int* pressed, bool ctrl_down )
{
  dbgvis_display_opts_t* display_opts = &ctx->display_opts;
  if( pressed['A'] && !ctrl_down ) 
  {
    display_opts->show_axis = !display_opts->show_axis; 
    pressed['A'] = 0;
  }
  if( pressed['R'] && !ctrl_down )
  {
    display_opts->show_grid = !display_opts->show_grid;
    pressed['R'] = 0; 
  }
}

void 
dbgvis__init_axes( mshgfx_geometry_t *axes_geo )
{
  int in_res = 12;
  float in_rad = 0.0085f;

  int res = msh_max(3, in_res);
  float rad = msh_max(0.0f, in_rad);

  // Cylinder
  int n_vertices = res * 2;
  int n_faces = 4 * res - 4;

  // Cone
  n_vertices += res+1;
  n_faces += 2*res-2; 

  msh_vec3_t* positions=(msh_vec3_t*)malloc(3*n_vertices*sizeof(msh_vec3_t));
  uint8_t* colors=(uint8_t*)malloc(3*n_vertices*4*sizeof(uint8_t));
  int* indices = (int*)malloc(3*3*n_faces*sizeof(int));
  
  // Cylinder
  int face_idx = 0;
  int pos_idx = 0;
  int col_idx = 0;
  int base_colors[] = { 31, 119, 180, 44, 160, 44, 214, 39, 40};
  int base_idx = 0;
  for( int j = 0 ; j < 3 ; ++j )
  {
    uint8_t cr = base_colors[3*j+0];
    uint8_t cg = base_colors[3*j+1];
    uint8_t cb = base_colors[3*j+2];
    
    base_idx = pos_idx;
    for(int i = 0; i < in_res; ++i)
    {
      float theta = (float)i/in_res * 6.28318530718f; /*2pi*/
      if(j == 0)
      {
        positions[pos_idx++] = msh_vec3(rad*sinf(theta), rad*cosf(theta), 0.0f);
        positions[pos_idx++] = msh_vec3(rad*sinf(theta), rad*cosf(theta), 0.85f);
      }
      if(j == 1)
      {
        positions[pos_idx++] = msh_vec3(rad*sinf(theta), 0.0f, rad*cosf(theta));
        positions[pos_idx++] = msh_vec3(rad*sinf(theta), 0.85f, rad*cosf(theta));
      }
      if( j == 2 )
      {
        positions[pos_idx++] = msh_vec3(0.0f, rad*sinf(theta), rad*cosf(theta));
        positions[pos_idx++] = msh_vec3(0.85f, rad*sinf(theta), rad*cosf(theta));
      }

      colors[col_idx++] = cr; colors[col_idx++] = cg; colors[col_idx++] = cb; colors[col_idx++] = 255; 
      colors[col_idx++] = cr; colors[col_idx++] = cg; colors[col_idx++] = cb; colors[col_idx++] = 255; 
      indices[face_idx++] = base_idx + (2*i);
      indices[face_idx++] = base_idx + (2*i+1);
      indices[face_idx++] = base_idx + ((2*(i+1)) % (2*res));

      indices[face_idx++] = base_idx + (2*i+1);
      indices[face_idx++] = base_idx + ((2*(i+1)) % (2*res));
      indices[face_idx++] = base_idx + ((2*(i+1)+1) % (2*res));
    }

    for(int i = 0; i < in_res-2; ++i)
    {
      indices[face_idx++] = base_idx + 0;
      indices[face_idx++] = base_idx + 2*i+2;
      indices[face_idx++] = base_idx + 2*i+4;
      indices[face_idx++] = base_idx + 1;
      indices[face_idx++] = base_idx + 2*i+3;
      indices[face_idx++] = base_idx + 2*i+5;
    }

    // Cone
    base_idx = pos_idx;
    for(int i = 0; i < in_res; ++i)
    {
      float theta = (float)i/in_res * 6.28318530718f; /*2pi*/
      if(j == 0)
      {
        positions[pos_idx++] = msh_vec3(2*rad*sinf(theta), 2*rad*cosf(theta), 0.85f);
      }
      if(j == 1)
      {
        positions[pos_idx++] = msh_vec3(2*rad*sinf(theta), 0.85f, 2*rad*cosf(theta));
      }
      if( j == 2 )
      {
        positions[pos_idx++] = msh_vec3(0.85f, 2*rad*sinf(theta), 2*rad*cosf(theta));
      }
      colors[col_idx++] = cr; ; colors[col_idx++] = cg; colors[col_idx++] = cb; colors[col_idx++] = 255; 
      indices[face_idx++] = base_idx + res;
      indices[face_idx++] = base_idx + i;
      indices[face_idx++] = base_idx + ((i+1) % (res));
    }
    if(j == 0)
    {
      positions[pos_idx++] = msh_vec3(0.0, 0.0, 1.0f);
    }
    if(j == 1)
    {
      positions[pos_idx++] = msh_vec3(0.0, 1.0f, 0.0);
    }
    if( j == 2 )
    {
      positions[pos_idx++] = msh_vec3(1.0f, 0.0, 0.0);
    }
    colors[col_idx++] = cr; colors[col_idx++] = cg; colors[col_idx++] = cb; colors[col_idx++] = 255; 

    for(int i = 0; i < in_res-2; ++i)
    {
      indices[face_idx++] = base_idx + 0;
      indices[face_idx++] = base_idx + i+1;
      indices[face_idx++] = base_idx + i+2;
    }
  }

  mshgfx_geometry_data_t axes_data;
  axes_data.positions  = (float*)&(positions[0]);
  axes_data.indices = (uint32_t*)&(indices[0]);
  axes_data.colors_a   = &(colors[0]);
  axes_data.n_vertices = pos_idx;
  axes_data.n_elements = face_idx;
  mshgfx_geometry_init(axes_geo, &axes_data, MSHGFX_POSITION | MSHGFX_COLOR_A | MSHGFX_STRUCTURED, MSHGFX_STATIC_DRAW); 
  free(positions);
  free(indices);
  free(colors);
}

void 
dbgvis_update_lines( mshgfx_geometry_t *lines_geo, msh_vec3_t* positions, uint8_t* colors, int n_points )
{
 //NOTE(maciej): Init/update lingo is kinda confusing.
 //NOTE(maciej): We don't know whether there will be more or less lines. Growing setup?
 if(lines_geo->vao <= 0 ) mshgfx_geometry_free(lines_geo);
 mshgfx_geometry_data_t lines_data;
 lines_data.positions = (float*)positions;
 lines_data.colors_a   = colors;
 lines_data.n_vertices = n_points;
 mshgfx_geometry_init(lines_geo, &lines_data, MSHGFX_POSITION | MSHGFX_COLOR_A, MSHGFX_DYNAMIC_DRAW); 
}


void 
dbgvis__init_grid( mshgfx_geometry_t *grid_geo, float size_x, float size_y )
{
  //NOTE(maciej): This only works for odd sizes
  int hsx = (int)floorf(size_x/2.0f);
  int hsy = (int)floorf(size_x/2.0f);
  int n_vertices = (int)(size_x * 2 + size_y * 2);
  float* positions = (float*)malloc( n_vertices*sizeof(float) * 3);
  uint8_t* colors = (uint8_t*)malloc( n_vertices*sizeof(uint8_t) * 4);
  int p_idx = 0, c_idx = 0;

  uint8_t bc[4] = {128, 128, 128, 255}; 
  for(int x = -hsx; x <= hsx; ++x)
  {
    float f = 1.0;
    if( (x+hsx) % 5 == 0 ) f = 0.5;
    if( x == 0 ) f = 0.25;
    
    positions[p_idx+0] = (float)x;             positions[p_idx+3]     = (float)x;
    positions[p_idx+1] = (float)0;             positions[p_idx+4]     = (float)0;
    positions[p_idx+2] = (float)-hsy;          positions[p_idx+5]     = (float)hsy;
    colors[c_idx+0]    = (uint8_t)(bc[0] * f); colors[c_idx+4] = (uint8_t)(bc[0] * f);
    colors[c_idx+1]    = (uint8_t)(bc[1] * f); colors[c_idx+5] = (uint8_t)(bc[1] * f);
    colors[c_idx+2]    = (uint8_t)(bc[2] * f); colors[c_idx+6] = (uint8_t)(bc[2] * f);
    colors[c_idx+3]    = 255;                  colors[c_idx+7]            = 255;
    p_idx += 6;
    c_idx += 8;
  }

  for(int y = -hsy; y <= hsy; ++y)
  {
    float f = 1.0;
    if( (y+hsy) % 5 == 0 ) f = 0.5;
    if( y == 0 ) f = 0.25;

    positions[p_idx+0] = (float)-hsx;       positions[p_idx+3] = (float)hsx;
    positions[p_idx+1] = 0;                 positions[p_idx+4] = 0;
    positions[p_idx+2] =  (float)y;         positions[p_idx+5] =  (float)y;
    colors[c_idx+0] = (uint8_t)(bc[0] * f); colors[c_idx+4] = (uint8_t)(bc[0] * f);
    colors[c_idx+1] = (uint8_t)(bc[1] * f); colors[c_idx+5] = (uint8_t)(bc[1] * f);
    colors[c_idx+2] = (uint8_t)(bc[2] * f); colors[c_idx+6] = (uint8_t)(bc[2] * f);
    colors[c_idx+3] = 255;                  colors[c_idx+7] = 255;
    p_idx += 6;
    c_idx += 8;
  }


  mshgfx_geometry_data_t grid_data;
  grid_data.positions  = &(positions[0]);
  grid_data.colors_a   = &(colors[0]);
  grid_data.n_vertices = p_idx / 3;
  mshgfx_geometry_init(grid_geo, &grid_data, MSHGFX_POSITION | MSHGFX_COLOR_A, MSHGFX_STATIC_DRAW); 
  free(positions);
  free(colors);
}

void dbgvis_render_axis(dbgvis_ctx_t* ctx, msh_mat4_t pose)
{
  dbgvis_display_opts_t* display_opts = &ctx->display_opts;
  if(display_opts->show_axis)
  {
    glDisable(GL_CULL_FACE);
    mshgfx_shader_prog_t* line_shdr = &ctx->line_shdrs[(int)display_opts->shader_mode];  
    msh_mat4_t vp = msh_mat4_mul(ctx->camera->proj, ctx->camera->view);
    msh_mat4_t mvp = msh_mat4_mul(vp, pose);
    mshgfx_shader_prog_use(line_shdr);
    mshgfx_shader_prog_set_uniform_4fm(line_shdr, "mvp", &mvp);
    mshgfx_geometry_draw(&ctx->axes_geo, GL_TRIANGLES);
  }
}

void dbgvis_render_grid(dbgvis_ctx_t* ctx)
{
  dbgvis_display_opts_t* display_opts = &ctx->display_opts;
  if(display_opts->show_grid)
  {
    mshgfx_shader_prog_t* line_shdr = &ctx->line_shdrs[(int)display_opts->shader_mode];    
    glDisable(GL_CULL_FACE);
    msh_mat4_t vp = msh_mat4_mul(ctx->camera->proj, ctx->camera->view);
    msh_mat4_t model = msh_mat4_identity();
    msh_mat4_t mvp   = msh_mat4_mul(vp, model);
    mshgfx_shader_prog_use(line_shdr);

    mshgfx_shader_prog_set_uniform_1f(line_shdr, "line_width", display_opts->line_width);
    mshgfx_shader_prog_set_uniform_4fm(line_shdr, "mvp", &mvp );
    mshgfx_geometry_draw(&ctx->grid_geo, GL_LINES);
    glEnable(GL_CULL_FACE);
  }
}

void dbgvis_render_lines(dbgvis_ctx_t* ctx, mshgfx_geometry_t* lines_geo, GLenum render_type)
{
  dbgvis_display_opts_t* display_opts = &ctx->display_opts;
  if(display_opts->show_grid)
  {
    mshgfx_shader_prog_t* line_shdr = &ctx->line_shdrs[(int)display_opts->shader_mode];    
    msh_mat4_t vp = msh_mat4_mul(ctx->camera->proj, ctx->camera->view);
    msh_mat4_t model = msh_mat4_identity();
    msh_mat4_t mvp   = msh_mat4_mul(vp, model);
    mshgfx_shader_prog_use(line_shdr);
    mshgfx_shader_prog_set_uniform_1f(line_shdr, "line_width", display_opts->line_width);
    mshgfx_shader_prog_set_uniform_4fm(line_shdr, "mvp", &mvp );
    mshgfx_geometry_draw(lines_geo, render_type);
  }
}

void 
dbgvis_new_frame(dbgvis_ctx_t* ctx, msh_camera_t* cam)
{
  ctx->camera = cam;

  // Backup GL state
  dbgvis_ogl_state_t* ogl_state = &ctx->ogl_state_backup;
  glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&ogl_state->active_texture);
  glGetIntegerv(GL_CURRENT_PROGRAM, &ogl_state->active_program);
  glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &ogl_state->array_buffer);
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &ogl_state->vertex_array);
  glGetIntegerv(GL_BLEND_SRC_RGB, (GLint*)&ogl_state->blend_src_rgb);
  glGetIntegerv(GL_BLEND_DST_RGB, (GLint*)&ogl_state->blend_dst_rgb);
  glGetIntegerv(GL_BLEND_SRC_ALPHA, (GLint*)&ogl_state->blend_src_alpha);
  glGetIntegerv(GL_BLEND_DST_ALPHA, (GLint*)&ogl_state->blend_dst_alpha);
  glGetIntegerv(GL_BLEND_EQUATION_RGB, (GLint*)&ogl_state->blend_equation_rgb);
  glGetIntegerv(GL_BLEND_EQUATION_ALPHA, (GLint*)&ogl_state->blend_equation_alpha);
  ogl_state->enable_blend = glIsEnabled(GL_BLEND);
  ogl_state->enable_cull_face = glIsEnabled(GL_CULL_FACE);
  ogl_state->enable_depth_test = glIsEnabled(GL_DEPTH_TEST);
  ogl_state->enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST);

  // Setup opengl state
  glActiveTexture(GL_TEXTURE0);  
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);
  
}

void
dbgvis_end_frame(dbgvis_ctx_t *ctx)
{
  dbgvis_ogl_state_t* prev_ogl_state = &ctx->ogl_state_backup;

  glActiveTexture(prev_ogl_state->active_texture);
  glUseProgram(prev_ogl_state->active_program);
  glBindVertexArray(prev_ogl_state->vertex_array);
  glBindBuffer(GL_ARRAY_BUFFER, prev_ogl_state->array_buffer);
  glBlendFunc(prev_ogl_state->blend_src_alpha, 
              prev_ogl_state->blend_dst_alpha);
  if(prev_ogl_state->enable_blend)        glEnable(GL_BLEND); 
  else                                    glDisable(GL_BLEND);
  if(prev_ogl_state->enable_cull_face)    glEnable(GL_CULL_FACE);
  else                                    glDisable(GL_CULL_FACE);
  if(prev_ogl_state->enable_depth_test)   glEnable(GL_DEPTH_TEST);
  else                                    glDisable(GL_DEPTH_TEST);
  if(prev_ogl_state->enable_scissor_test) glEnable(GL_SCISSOR_TEST);
  else                                    glDisable(GL_SCISSOR_TEST);
}

// Forward declare of shader source setup. Sources for shaders are at the bottom
// of this file
void dbgvis__init_shader_src_code(dbgvis_ctx_t* ctx);

int dbgvis_init_context(dbgvis_ctx_t* ctx)
{
  dbgvis__init_shader_src_code(ctx);

  // Shader compilation
  mshgfx_shader_prog_create_from_source_vf(&ctx->line_shdrs[0], 
                                            ctx->shader_source.flat_vs, 
                                            ctx->shader_source.flat_fs);
  // mshgfx_shader_prog_create_from_source_vgf(&ctx->line_shdrs[1], 
  //                                            ctx->shader_source.flat_vs, 
  //                                            ctx->shader_source.line_gs, 
  //                                            ctx->shader_source.flat_fs);
  
  // Default view options
  ctx->display_opts.color_mode          = DBGVIS_FLAT_COLOR;
  ctx->display_opts.shader_mode         = DBGVIS_LINE;
  ctx->display_opts.line_width          = 1.0f;
  ctx->display_opts.show_axis           = 1;
  ctx->display_opts.axis_scale          = 3.0f;
  ctx->display_opts.show_grid           = 1;
  ctx->display_opts.grid_scale          = 2.0f;
  
  dbgvis__init_axes(&ctx->axes_geo);
  dbgvis__init_grid(&ctx->grid_geo, 11, 11);

  return 1;
}


int dbgvis_destroy_context(dbgvis_ctx_t* ctx)
{
  return 1;
}




////////////////////////////////////////////////////////////////////////////////
// GPU Code
////////////////////////////////////////////////////////////////////////////////

void 
dbgvis__init_shader_src_code(dbgvis_ctx_t* ctx)
{

ctx->shader_source.flat_vs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
    layout(location = 0) in vec3 position;
    layout(location = 4) in vec4 color;
    uniform mat4 mvp;
    
    out vec4 v_color;
    void main() {
      gl_Position = mvp * vec4(position, 1.0);
      gl_PointSize = 10.0f;
      v_color = color;
    });
  
ctx->shader_source.flat_fs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
    layout (location = 0) out vec4 frag_color;
    in vec4 v_color;
    void main() {
      frag_color = v_color;
    });
}

#endif /*DEBUGVIS_IMPLEMENTATION*/