/*
  ==============================================================================
  
  CLOUDVIS.H - WIP! 
  
  This header file provides pointcloud rendering functionality, built a top
  of mshgfx mostly. Currently implements:
  1. Simple pointcloud rendering
  2. Pointcloud rendering as surfels using geometry shaders.

  Each point can have following attributes:
  - Position
  - Normal
  - Color
  - Radius
  - Type/Class index.
  - User Data

  User data allows user to specify xxx values per point to descibe different 
  quantities per pixels. 

  #define CLOUDVIS_IMPLEMENTATION
  #include "CLOUDIVS.h"

  ==============================================================================
  DEPENDENCIES
    msh.h
    msh_vec_math.h
    msh_gfx.h
    mshgfx_geometry.h
    
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
  TODOs:
  [ ] Delete programs on shutdown!!
  [x] EDL
  [ ] Splatting
  [x] Remove basic rendering of basic stuff
  [ ] Make point culling optional.
  
  ==============================================================================
  REFERENCES:
*/
#ifndef CLOUDVIS_H
#define CLOUDVIS_H


typedef enum cldvis_shader_mode
{
  CLDVIS_POINT     = 0,
  CLDVIS_SURFEL    = 1,
  CLDVIS_N_SHDR_MODES,
  CLDVIS_SPLAT     
} cldvis_shader_mode_t;

typedef enum cldvis_color_mode
{
  CLDVIS_TEX_COLOR      = 0,
  CLDVIS_FLAT_COLOR,
  CLDVIS_HEIGHT,
  CLDVIS_NORMAL,
  CLDVIS_INSTANCE_ID,
  CLDVIS_CLASS_ID,
  CLDVIS_QUALITY,
  CLDVIS_PHONG,
  CLDVIS_UNIFORM_ID,
  CLDVIS_N_CLR_MODES
} cldvis_color_mode_t;

typedef struct cloudvis_display_options
{
  msh_vec4_t viewport;
  cldvis_shader_mode_t shader_mode;
  cldvis_color_mode_t  color_mode;
  int32_t show_normals;

  float surfel_size;
  float max_depth;
  int culling_enabled;
  float normal_length;
  float fovy;

  int32_t edl_use;
  float edl_strength;
  float edl_radius;

  int32_t modify_depth_buffer;
} cldvis_display_opts_t;

typedef struct cldvis_ogl_state
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
} cldvis_ogl_state_t;

typedef struct cldvis_shader_sources
{
  char *screen_vs, *present_fs;
  char *point_vs, *point_fs;
  char *surfel_vs, *surfel_gs, *surfel_fs;
  char *normal_gs, *normal_fs;
} cldvis_shader_sources_t;

typedef struct cldvis_ctx
{
  //TODO(maciej): Do it more like g-buffer? i.e. packing? Check performance
  //TODO(maciej): Look into linearizing depth buffer
  GLuint present_vao;  
  mshgfx_geometry_t screen_geo;  
  mshgfx_framebuffer_t deffered_stage;
  mshgfx_texture2d_t final_color_buffer, final_depth_buffer;
  
  mshgfx_shader_prog_t present_shdr;
  mshgfx_shader_prog_t cloud_shdrs[3];
  mshgfx_shader_prog_t normal_shdr;

  msh_camera_t* camera;

  cldvis_display_opts_t display_opts;
  cldvis_ogl_state_t ogl_state_backup;
  cldvis_shader_sources_t shader_source;
} cldvis_ctx_t;

// Rendering functions
void cldvis_render_cloud(cldvis_ctx_t* ctx, 
                         mshgfx_geometry_t* pointcloud, msh_mat4_t* pose, 
                         int32_t data,
                         msh_bbox_t* bbox );

void cldvis_render_clouds(cldvis_ctx_t* ctx, 
                          mshgfx_geometry_t* pointclouds, msh_mat4_t* poses, 
                          msh_bbox_t* bboxes, int8_t* visibility, int32_t* data,
                          int32_t n_clouds);

void cldvis_render_clouds_range(cldvis_ctx_t* ctx, 
                          mshgfx_geometry_t* pointclouds, msh_mat4_t* poses, 
                          msh_bbox_t* bboxes, int8_t* visibility, int32_t* data,
                          int32_t start_idx, int32_t end_idx);

void cldvis_resize(cldvis_ctx_t* ctx, msh_vec4_t viewport);
void cldvis_new_frame(cldvis_ctx_t* ctx, msh_camera_t* cam);
void cldvis_end_frame(cldvis_ctx_t *ctx);
void cldvis_present(cldvis_ctx_t* ctx);

int cldvis_init_context(cldvis_ctx_t* ctx);
int cldvis_destroy_context(cldvis_ctx_t* ctx);

void cldvis_keyboard_shortcuts( cldvis_ctx_t* ctx, int* pressed, bool ctrl_down );
#ifdef CLOUDVIS_USE_NK_GUI
void cldvis_nk_gui(cldvis_ctx_t* cloudvis_ctx, struct nk_context *nk_ctx);
#endif

#endif /*CLOUDVIS_H*/


#ifdef CLOUDVIS_IMPLEMENTATION


////////////////////////////////////////////////////////////////////////////////
// Cloudvis Rendering
////////////////////////////////////////////////////////////////////////////////

// These are tableau colors
#define CLDVIS__NCOLORS 40
static const int32_t CLDVIS__COLOR_TABLE[CLDVIS__NCOLORS] = 
{ 0x1f77b4, 0xaec7e8, 0xff7f0e, 0xffbb78, 
  0x2ca02c, 0x98df8a, 0xd62728, 0xff9896,
  0x9467bd, 0xc5b0d5, 0x8c564b, 0xc49c94,
  0xe377c2, 0xdbdb8d, 0xf7b6d2, 0xbcbd22, 
  0x17becf, 0xffe94f, 0x9edae5, 0xcc2b85,
  0xff524f, 0x2e7bb2, 0x9343db, 0x25b258,
  0xb27f2e, 0x337a29, 0xf24fa1, 0x4287cb,
  0xf2b273, 0xf2de37, 0x8080ff, 0xfffe59,
  0x47908b, 0x8856a7, 0x9ebcda, 0xe0ecf4,
  0xffeda0, 0xfeb24c, 0xf03b20, 0xbdbdbd };

msh_vec4_t cldvis__hex2color(int32_t hex)
{
  uint8_t r = (hex >> 16 & 0x0000FF);
  uint8_t g = (hex >>  8 & 0x0000FF);
  uint8_t b = (hex >>  0 & 0x0000FF);
  return msh_vec4(r/255.0f, g/255.0f, b/255.0f, 1.0f);
}

#ifdef CLOUDVIS_USE_NK_GUI
void 
cldvis_nk_gui(cldvis_ctx_t* cldvis_ctx, struct nk_context *nk_ctx)
{
  cldvis_display_opts_t* opts = &cldvis_ctx->display_opts; 

  // Shader type
  nk_layout_row_dynamic(nk_ctx, 20, 1);
  nk_label(nk_ctx, "Shader Type:", NK_TEXT_ALIGN_LEFT | NK_TEXT_ALIGN_BOTTOM);
  int shdr_idx = (int)opts->shader_mode;
  static const char *shdr_labels[] = {"Point","Surfel"};
  nk_layout_row_static(nk_ctx, 15, 175, 1);
  shdr_idx = nk_combo(nk_ctx, shdr_labels, msh_count_of(shdr_labels), 
                          shdr_idx, 15, nk_vec2(175,175) );
  opts->shader_mode = (cldvis_shader_mode_t) shdr_idx;
  
  // Surfel Coloring
  nk_layout_row_dynamic(nk_ctx, 20, 1);
  nk_label(nk_ctx, "Color Type:", NK_TEXT_ALIGN_LEFT | NK_TEXT_ALIGN_BOTTOM);
  
  int clr_idx = opts->color_mode;
  static const char *clr_labels[] = {"Tex Color", "Flat Color", "Height",
                                      "Normal", "InstanceID", "ClassID", 
                                      "Quality", "Phong", "UniformID"};
  nk_layout_row_static(nk_ctx, 15, 175, 1);
  clr_idx = nk_combo(nk_ctx, clr_labels, msh_count_of(clr_labels), 
                          clr_idx, 15, nk_vec2(175,200) );
  opts->color_mode = (cldvis_color_mode_t)clr_idx;
  
  // Misc view options. Add to collapsing tree?
  nk_layout_row_dynamic(nk_ctx, 25, 1);
  nk_label(nk_ctx, "View Options:", NK_TEXT_ALIGN_LEFT | NK_TEXT_ALIGN_BOTTOM);
  nk_layout_row_dynamic(nk_ctx, 15, 1);
  nk_checkbox_label(nk_ctx, "Show Normals", &opts->show_normals);
  
  nk_layout_row_dynamic(nk_ctx, 15, 1);
  nk_property_float(nk_ctx, "Surfel size", 0.1f, &opts->surfel_size, 100.0f, 0.05f, 0.05f);
  nk_property_float(nk_ctx, "Normal length", 1.0f, &opts->normal_length, 100.0f, 0.5f, 0.5f);
  nk_property_float(nk_ctx, "Max. Depth", 1.0f, &opts->max_depth, 20.0f, 0.05f, 0.05f);
  nk_property_float(nk_ctx, "FovY", 0.05f, &opts->fovy, 2.0f, 0.05f, 0.01f);
  nk_checkbox_label(nk_ctx, "Enable Culling", &opts->culling_enabled);

  nk_checkbox_label(nk_ctx, "EDL", &opts->edl_use);
  nk_property_float(nk_ctx, "EDL Strength", 0.5f, &opts->edl_strength, 10.0f, 0.05f, 0.05f);
  nk_property_float(nk_ctx, "EDL Radius", 0.5f, &opts->edl_radius, 10.0f, 0.05f, 0.05f);

}
#endif

void 
cldvis_keyboard_shortcuts( cldvis_ctx_t* ctx, int* pressed, bool ctrl_down )
{
  cldvis_display_opts_t* display_opts = &ctx->display_opts;
  if( pressed['C'] && !ctrl_down ) 
  {
    int mode = ((display_opts->color_mode + 1) % CLDVIS_N_CLR_MODES );
    display_opts->color_mode = (cldvis_color_mode_t)mode;
    pressed['C'] = 0; 
  }
  if( pressed['E'] && !ctrl_down )
  {
    display_opts->edl_use = !display_opts->edl_use;
    pressed['E'] = 0; 
  }
  if( pressed['N'] && !ctrl_down ) 
  {
    display_opts->show_normals = !(display_opts->show_normals);
    pressed['N'] = 0; 
  }
  if( pressed['V'] && !ctrl_down ) 
  {
    int mode = ((display_opts->shader_mode + 1) % CLDVIS_N_SHDR_MODES );
    display_opts->shader_mode = (cldvis_shader_mode_t)mode;
    pressed['V'] = 0; 
  }
}

void 
cldvis_render_clouds_range(cldvis_ctx_t* ctx, 
                           mshgfx_geometry_t* pointclouds, msh_mat4_t* poses, 
                           msh_bbox_t* bboxes, int8_t* visibility, int32_t* data,
                           int32_t start_idx, int32_t end_idx)
{
  //TODO(maciej): Add assertion checking that end_idx <= n_pointclouds
  //TODO(maciej): Add assertion on pointcloud and poses


  // Get data from context
  cldvis_display_opts_t* display_opts = &ctx->display_opts;
  msh_mat4_t vp  = msh_mat4_mul(ctx->camera->proj, ctx->camera->view);

  mshgfx_shader_prog_t cld_shdr = ctx->cloud_shdrs[(int32_t)display_opts->shader_mode];

  if(display_opts->culling_enabled) { glEnable(GL_CULL_FACE); }
  else                              { glDisable(GL_CULL_FACE);}

  // Render pointclouds
  mshgfx_shader_prog_use(&cld_shdr);
  mshgfx_shader_prog_set_uniform_1f(&cld_shdr, "point_size", display_opts->surfel_size);
  mshgfx_shader_prog_set_uniform_1f(&cld_shdr, "max_depth", display_opts->max_depth);
  mshgfx_shader_prog_set_uniform_1i(&cld_shdr, "culling_enabled", display_opts->culling_enabled);
  mshgfx_shader_prog_set_uniform_1i(&cld_shdr, "color_mode", display_opts->color_mode);
  
  msh_vec3_t min_p = msh_vec3(-1.0f, -1.0f, -1.0f);
  msh_vec3_t max_p = msh_vec3( 1.0f,  1.0f,  1.0f);
  for(int32_t idx = start_idx; idx < end_idx; ++idx)
  {
    if(visibility && *(visibility+idx) == 0) continue;
    msh_mat4_t pose = poses[idx];
    msh_mat4_t mv  = msh_mat4_mul(ctx->camera->view, pose);
    msh_mat4_t mvp = msh_mat4_mul(vp, pose);
    mshgfx_geometry_t* cloud = pointclouds + idx;

    msh_vec4_t flat_color = cldvis__hex2color(CLDVIS__COLOR_TABLE[idx % CLDVIS__NCOLORS]);
    if(bboxes)
    {
      min_p = (bboxes+idx)->min_p;
      max_p = (bboxes+idx)->max_p;
    }
    mshgfx_shader_prog_set_uniform_4fv(&cld_shdr, "flat_color", &flat_color);
    mshgfx_shader_prog_set_uniform_4fm(&cld_shdr, "mvp",   &mvp);
    mshgfx_shader_prog_set_uniform_4fm(&cld_shdr, "mv",    &mv);
    mshgfx_shader_prog_set_uniform_4fm(&cld_shdr, "model", &pose);
    mshgfx_shader_prog_set_uniform_4fv(&cld_shdr, "viewport", &display_opts->viewport);
    mshgfx_shader_prog_set_uniform_3fv(&cld_shdr, "bbox_minp", &min_p);
    mshgfx_shader_prog_set_uniform_3fv(&cld_shdr, "bbox_maxp", &max_p);
    if(data) mshgfx_shader_prog_set_uniform_1i(&cld_shdr, "uniform_id", data[idx]);
    mshgfx_geometry_draw(cloud, GL_POINTS);
  }

  // Render Normals(optionally)
  if(display_opts->show_normals)
  {
    mshgfx_shader_prog_use(&ctx->normal_shdr);
    mshgfx_shader_prog_set_uniform_1f(&ctx->normal_shdr, "normal_length", display_opts->normal_length);
    mshgfx_shader_prog_set_uniform_1i(&ctx->normal_shdr, "color_mode",    display_opts->color_mode);
    for( int idx = start_idx; idx < end_idx; ++idx )
    {
      if(visibility && *(visibility+idx) == 0) continue;
      msh_mat4_t pose = poses[idx];
      msh_mat4_t mvp = msh_mat4_mul(vp, pose);
      mshgfx_geometry_t* cloud = pointclouds + idx;
      mshgfx_shader_prog_set_uniform_1i(&ctx->normal_shdr, "idx", idx);
      mshgfx_shader_prog_set_uniform_4fm(&ctx->normal_shdr, "model", &pose);
      mshgfx_shader_prog_set_uniform_4fm(&ctx->normal_shdr, "mvp", &mvp);
      mshgfx_geometry_draw(cloud, GL_POINTS);
    }
  }
}

void 
cldvis_render_clouds( cldvis_ctx_t* ctx, 
                      mshgfx_geometry_t* pointclouds, msh_mat4_t* poses, 
                      msh_bbox_t* bboxes, int8_t* visibility, int32_t* data,
                      int32_t n_clouds )
{
  cldvis_render_clouds_range(ctx, pointclouds, poses, 
                             bboxes, visibility, data,
                             0, n_clouds );
}

void cldvis_render_cloud(cldvis_ctx_t* ctx, 
                         mshgfx_geometry_t* pointcloud, msh_mat4_t* pose, 
                         int32_t data,
                         msh_bbox_t* bbox )
{
  cldvis_render_clouds_range(ctx, pointcloud, pose, bbox, NULL, &data, 0, 1);
}

void
cldvis_resize(cldvis_ctx_t* ctx, msh_vec4_t viewport)
{
  int w = (int)viewport.z;
  int h = (int)viewport.w;
  ctx->display_opts.viewport = viewport;
  
  mshgfx_texture2d_free(&ctx->final_color_buffer);
  mshgfx_texture2d_free(&ctx->final_depth_buffer);

  mshgfx_texture2d_init(&ctx->final_color_buffer, NULL, GL_UNSIGNED_BYTE, 
                            w, h, 4, 1, MSHGFX_LINEAR | MSHGFX_MIRRORED_REPEAT);
  mshgfx_texture2d_init(&ctx->final_depth_buffer, NULL, GL_DEPTH_COMPONENT, 
                           w, h, 1, 2, MSHGFX_NEAREST | MSHGFX_MIRRORED_REPEAT);

  GLuint color_attachments[] = { GL_COLOR_ATTACHMENT0 };
  GLuint depth_attachments[] = { GL_DEPTH_ATTACHMENT };

  if(ctx->deffered_stage.id > 0 ) mshgfx_framebuffer_free(&ctx->deffered_stage);
  mshgfx_framebuffer_init(&ctx->deffered_stage, w, h);
  mshgfx_framebuffer_attach_textures(&ctx->deffered_stage, 
                                &ctx->final_color_buffer, color_attachments, 1);
  mshgfx_framebuffer_attach_textures(&ctx->deffered_stage, 
                                &ctx->final_depth_buffer, depth_attachments, 1);
  mshgfx_framebuffer_check_status(&ctx->deffered_stage);
  mshgfx_framebuffer_bind(0);
}

void 
cldvis_new_frame(cldvis_ctx_t* ctx, msh_camera_t* cam)
{
  cldvis_display_opts_t* disp_opts = &ctx->display_opts;

  // Backup GL state
  cldvis_ogl_state_t* ogl_state = &ctx->ogl_state_backup;
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

  // Setup openg state
  glActiveTexture(GL_TEXTURE0);  
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);
  
  //Prepare for subsequent rendering
  ctx->camera = cam;
  glViewport((GLsizei)disp_opts->viewport.x, (GLsizei)disp_opts->viewport.y, 
             (GLsizei)disp_opts->viewport.z, (GLsizei)disp_opts->viewport.w);
  mshgfx_framebuffer_bind(&ctx->deffered_stage);
  mshgfx_background_flat4f( 0.0f, 0.0f, 0.0f, 0.0f);
}

void
cldvis_end_frame(cldvis_ctx_t *ctx)
{
  cldvis_ogl_state_t* prev_ogl_state = &ctx->ogl_state_backup;
  mshgfx_framebuffer_bind(0);

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


// NOTE(maciej):cldvis_present is different from end frame as we might only
// want to draw pointcloud to rgbd framebuffer and process it different, without
// ever drawing it to default framebuffer.
void 
cldvis_present(cldvis_ctx_t* ctx)
{
  int w = (int)ctx->display_opts.viewport.z;
  int h = (int)ctx->display_opts.viewport.w;

  mshgfx_texture2d_use(&ctx->final_color_buffer);
  mshgfx_texture2d_use(&ctx->final_depth_buffer);
  mshgfx_shader_prog_use(&ctx->present_shdr);
  mshgfx_shader_prog_set_uniform_1i(&ctx->present_shdr, "color_tex", 
                                                  ctx->final_color_buffer.unit);
  mshgfx_shader_prog_set_uniform_1i(&ctx->present_shdr, "depth_tex", 
                                                  ctx->final_depth_buffer.unit);
  mshgfx_shader_prog_set_uniform_2f(&ctx->present_shdr, "screen_res", 
                                                            (float)w, (float)h);
  mshgfx_shader_prog_set_uniform_1i(&ctx->present_shdr, "edl_use", 
                                                     ctx->display_opts.edl_use);
  mshgfx_shader_prog_set_uniform_2f(&ctx->present_shdr, "edl_params", 
                                                ctx->display_opts.edl_radius,
                                                ctx->display_opts.edl_strength);

  //TODO(maciej): Tie this to mshgfx somehow?
  glBindVertexArray(ctx->present_vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);

  // Only if modify_depth_buffer is active
  // NOTE(maciej): This only works if default and cldvis depth buffer formats agree.
  //               Is there a way to check this?
  if( ctx->display_opts.modify_depth_buffer )
  {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, ctx->deffered_stage.id);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer( 0, 0, w, h, 0, 0, w, h, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
}

// Forward declare of shader source setup. Sources for shaders are at the bottom
// of this file
void cldvis__init_shader_src_code(cldvis_ctx_t* ctx);

int cldvis_init_context(cldvis_ctx_t* ctx)
{
  glGenVertexArrays(1, &ctx->present_vao);
  cldvis__init_shader_src_code(ctx);

  // Shader compilation
  mshgfx_shader_prog_create_from_source_vf(&ctx->cloud_shdrs[0], 
                                            ctx->shader_source.point_vs, 
                                            ctx->shader_source.point_fs);
  mshgfx_shader_prog_create_from_source_vgf(&ctx->cloud_shdrs[1], 
                                            ctx->shader_source.surfel_vs, 
                                            ctx->shader_source.surfel_gs, 
                                            ctx->shader_source.surfel_fs);
  mshgfx_shader_prog_create_from_source_vgf(&ctx->normal_shdr, 
                                            ctx->shader_source.surfel_vs, 
                                            ctx->shader_source.normal_gs, 
                                            ctx->shader_source.normal_fs);
  mshgfx_shader_prog_create_from_source_vf(&ctx->present_shdr,
                                            ctx->shader_source.screen_vs,
                                            ctx->shader_source.present_fs);
  
  // Default view options
  ctx->display_opts.color_mode          = CLDVIS_TEX_COLOR;
  ctx->display_opts.shader_mode         = CLDVIS_POINT;
  ctx->display_opts.surfel_size         = 1.0f;
  ctx->display_opts.normal_length       = 5.0f;
  ctx->display_opts.edl_use             = 0;
  ctx->display_opts.edl_strength        = 0.4;
  ctx->display_opts.edl_radius          = 1.4;
  ctx->display_opts.show_normals        = 0;
  ctx->display_opts.max_depth           = 5.0f;
  ctx->display_opts.culling_enabled     = 1;
  ctx->display_opts.modify_depth_buffer = 1;

  return 1;
}

int cldvis_destroy_context(cldvis_ctx_t* ctx)
{

  mshgfx_shader_prog_delete(&ctx->present_shdr);
  mshgfx_shader_prog_delete(&ctx->cloud_shdrs[0]);
  mshgfx_shader_prog_delete(&ctx->cloud_shdrs[1]);
  mshgfx_shader_prog_delete(&ctx->cloud_shdrs[2]);
  mshgfx_shader_prog_delete(&ctx->normal_shdr);
  glDeleteVertexArrays(1, &ctx->present_vao);
  return 1;
}


////////////////////////////////////////////////////////////////////////////////
// GPU Code
////////////////////////////////////////////////////////////////////////////////

void 
cldvis__init_shader_src_code(cldvis_ctx_t* ctx)
{
// vertex shader (full screen quad by Morgan McGuire)
ctx->shader_source.screen_vs = (char*)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  void main()
  {
    uint idx = uint( gl_VertexID );
    gl_Position = vec4(
        (float( idx &  1U ) ) * 4.0 - 1.0,
        (float( idx >> 1U  ) ) * 4.0 - 1.0,
        0.0, 1.0);
  });

// NOTE(maciej): Try implementing version of AO, see how it is performing
// NOTE(maciej): Defer everything, reduce overdraw in shading. This means modifying the
// Point and surfel shaders;
// TODO(maciej): Render linear depths somehow? Or linearize dephts here? The current edl is most likely bogus.
// TODO(maciej): Uniform blocks
ctx->shader_source.present_fs = (char*)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  uniform sampler2D color_tex;
  uniform sampler2D depth_tex;
  uniform vec2 screen_res;

  uniform bool edl_use;
  uniform float near;
  uniform float far;
  uniform vec2 edl_params;

  //TODO(maciej): Pass camera near/far planes
  float 
  linearize_depth( float d )
  {
    float f = 500.0;
    float n = 0.1;
    return ((2.0 * n) / (f + n - d * (f - n)));
  }

  float
  sample_depth( vec2 uv )
  {
    return linearize_depth( texture(depth_tex, uv).r);
  }

  out vec4 frag_color;
  void main()
  {
    vec2 uv = gl_FragCoord.xy / screen_res;
    
    vec4 color = texture(color_tex, uv);
    float edl_shade = 1.0;
    float edl_alpha = 0.0;
    if( edl_use )
    {
      float edl_radius = edl_params.x;
      float edl_str = edl_params.y;
      float x_step = edl_radius / screen_res.x;
      float y_step = edl_radius / screen_res.y;
      float d_c = log2(sample_depth(uv));
      float d_n = log2(sample_depth(uv+vec2(     0.0,  y_step)));
      float d_s = log2(sample_depth(uv+vec2(     0.0, -y_step)));
      float d_w = log2(sample_depth(uv+vec2( -x_step,     0.0)));
      float d_e = log2(sample_depth(uv+vec2(  x_step,     0.0)));
      
      float response = 0.0;
      response += max(0, d_c - d_n);
      response += max(0, d_c - d_s); 
      response += max(0, d_c - d_e); 
      response += max(0, d_c - d_w); 
      response /= 4.0;

      edl_shade = exp(-response * 300.0 * edl_str);
      edl_alpha = float(response > 0);
    }
    frag_color = vec4(color.rgb * edl_shade, clamp(color.a + edl_alpha, 0.0, 1.0));
  });


#define CLDVIS_SHADER_CONSTANTS MSHGFX_SHADER_STRINGIFY(\
  const int CLDVIS_TEX_COLOR      = 0;\
  const int CLDVIS_FLAT_COLOR     = 1;\
  const int CLDVIS_HEIGHT         = 2;\
  const int CLDVIS_NORMAL         = 3;\
  const int CLDVIS_INSTANCE_ID        = 4;\
  const int CLDVIS_CLASS_ID     = 5;\
  const int CLDVIS_QUALITY        = 6;\
  const int CLDVIS_PHONG          = 7;\
  const int CLDVIS_UNIFORM_ID     = 8;\
\
  const int CLDVIS_COLOR_TABLE[40] = int[40]\
  ( 0x1f77b4, 0xaec7e8, 0xff7f0e, 0xffbb78, \
    0x2ca02c, 0x98df8a, 0xd62728, 0xff9896, \
    0x9467bd, 0xc5b0d5, 0x8c564b, 0xc49c94, \
    0xe377c2, 0xdbdb8d, 0xf7b6d2, 0xbcbd22, \
    0x17becf, 0xffe94f, 0x9edae5, 0xcc2b85, \
    0xff524f, 0x2e7bb2, 0x9343db, 0x25b258, \
    0xb27f2e, 0x337a29, 0xf24fa1, 0x4287cb, \
    0xf2b273, 0xf2de37, 0x8080ff, 0xfffe59, \
    0x47908b, 0x8856a7, 0x9ebcda, 0xe0ecf4, \
    0xffeda0, 0xfeb24c, 0xf03b20, 0xbdbdbd);\
\
  vec4 cldvis_hex2color(int hex)\
  {\
    int r = (hex >> 16 & 0x0000FF);\
    int g = (hex >>  8 & 0x0000FF);\
    int b = (hex >>  0 & 0x0000FF);\
    return vec4(r/255.0f, g/255.0f, b/255.0f, 1.0f);\
  })

ctx->shader_source.point_vs = (char *)MSHGFX_SHADER_HEAD CLDVIS_SHADER_CONSTANTS MSHGFX_SHADER_STRINGIFY(
  layout(location = 0) in vec3 position;
  layout(location = 1) in vec3 normal;
  layout(location = 4) in vec4 color_a;
  layout(location = 8) in vec4 user_data_a;

  uniform mat4 model;
  uniform mat4 mv;
  uniform mat4 mvp;
  uniform vec4 viewport;
  uniform vec4 flat_color;
  uniform vec3 bbox_minp;
  uniform vec3 bbox_maxp;
  uniform float max_depth;
  uniform int color_mode;
  uniform int culling_enabled;
  uniform float point_size; // this should be scaling
  uniform int uniform_id;
  
  out vec4 v_color;
  out vec3 v_normal;
  out vec3 v_world_pos;
  
  void main() {
    gl_Position = mvp * vec4(position, 1.0);
    // mvp maps to (-1, -1, -1, 1, 1, 1) cube, if outside depth range, put 
    // the point outside the cube.
    if( position.y > max_depth)
    {
      gl_Position = vec4(-2.0, -2.0, -2.0, 1);
      return;
    }
    v_normal = normal;
    float v_radius = user_data_a.r;
    float quality = user_data_a.a;
    float slope = tan(0.4); // need to pass fov in radians 
    float half_height = (viewport.w/2); // need to pass viewport 

    // Calculate lighting
    vec3 P = vec3(mv * vec4(position, 1.0));
    vec3 N = vec3(mv * vec4(normal, 0.0));
    vec3 V = -P / length(P);
    vec3 L = V;
    vec3 H = normalize( L + V);
    float ndotl = dot(N, L);
    float ndoth = dot(N, H);

    //Discard if need be
    if(culling_enabled != 0 && dot(N,V) < 0.0)
    {
      gl_Position = vec4(1,0,0,0);
      return;
    }
    // Calculate point size
    gl_PointSize = 1.65 * point_size * ((v_radius*half_height)/(slope*abs(P.z)));
    
    // Calculate color
    float shade = clamp( ndotl + 0.2 * pow(ndoth, 60.0), 0, 1);
    switch(color_mode)
    {
      case CLDVIS_TEX_COLOR:
        v_color = color_a;
        break;
      case CLDVIS_FLAT_COLOR:
        v_color = flat_color;
        break;
      case CLDVIS_HEIGHT:
        float range = bbox_maxp.y - bbox_minp.y;
        float height = position.y - bbox_minp.y;
        float hf = height/range;
        v_color = (1-hf)*vec4(0.0, 0.027, 0.698, 1.0) + hf*vec4(1.0, 0.824, 0.098, 1.0);
        break;
      case CLDVIS_NORMAL:
        v_color = vec4((vec3(model*vec4(normal, 0.0))+1.0) * 0.5,1.0);
        break;
      case CLDVIS_INSTANCE_ID:
        int type_id = int(user_data_a.g);
        v_color = cldvis_hex2color(CLDVIS_COLOR_TABLE[type_id%40]);
        if( type_id == 1024 ) { v_color = cldvis_hex2color( 0xff00ff ); }
        break;
      case CLDVIS_CLASS_ID:
        int segment_id = int(user_data_a.b);
        v_color = cldvis_hex2color(CLDVIS_COLOR_TABLE[segment_id%40]);
        if( segment_id == 0 ) { v_color = cldvis_hex2color( 0xff00ff ); }
        break;
      case CLDVIS_QUALITY:
        // TODO(maciej): Change this to divergent colors;
        vec4 col_a = vec4(0.1, 0.6, 0.9, 1.0);
        vec4 col_b = vec4(0.8, 0.2, 0.1, 1.0);
        v_color = quality*col_a + (1.0-quality)*col_b;
        break;
      case CLDVIS_UNIFORM_ID:
        v_color = cldvis_hex2color(CLDVIS_COLOR_TABLE[uniform_id%40]);
        if( uniform_id == 1024 ) { v_color = cldvis_hex2color( 0xff00ff ); }
        break;
      case CLDVIS_PHONG:
        float t = ndotl;
        vec3 c_a = vec3(0.76, 0.59, 0.44);
        vec3 c_b = vec3(0.38, 0.28, 0.21);
        vec3 c1 = t*c_a + (1.0-t)*c_b;
        vec3 c2 = 0.6 * pow(ndoth, 160.0) * vec3(0.95, 0.95, 0.8);
        vec3 c  = clamp(c1 + c2, 0, 1);
        v_color = vec4(c, 1.0);
        break;
      default:
        v_color = color_a;
    }  
  });

ctx->shader_source.point_fs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  in vec4 v_color;
  out vec4 frag_color;
  void main() {
    vec2 uv = gl_PointCoord * 2.0 - 1.0;
    float radius = sqrt(uv.x * uv.x + uv.y * uv.y);
    if(radius > 1.0) discard;
    frag_color = v_color;
  });

//TODO(maciej): Decide whether it is better to do coloring on cpu and upload
// or decode from user data in the shader.
ctx->shader_source.surfel_vs = (char *)MSHGFX_SHADER_HEAD CLDVIS_SHADER_CONSTANTS MSHGFX_SHADER_STRINGIFY(
  layout(location = 0) in vec3 position;
  layout(location = 1) in vec3 normal;
  layout(location = 4) in vec4 color_a;
  layout(location = 8) in vec4 user_data_a;

  uniform mat4 model;
  uniform mat4 mv;
  uniform vec4 viewport;
  uniform vec4 flat_color;
  uniform vec3 bbox_minp;
  uniform vec3 bbox_maxp;
  uniform int color_mode;
  uniform int idx;
  uniform int uniform_id;

  out vec4 v_color;
  out vec3 v_normal;
  flat out float v_point_depth;
  flat out float v_radius;

  void main() {
    gl_Position = vec4(position, 1.0);
    v_point_depth = position.y;//-position.z;
    v_normal = normal;
    float quality = user_data_a.a;
    v_radius = user_data_a.r;

    // Calculate lighting
    vec3 P = vec3(mv * vec4(position, 1.0));
    vec3 N = vec3(mv * vec4(normal, 0.0));
    vec3 V = -P / length(P);
    vec3 L = V;
    vec3 H = normalize( L + V);
    float ndotl = dot(N, L);
    float ndoth = dot(N, H);
    float shade = clamp( ndotl + 0.2 * pow(ndoth, 60.0), 0, 1);
    switch(color_mode)
    {
      case CLDVIS_TEX_COLOR:
        v_color = color_a;
        break;
      case CLDVIS_FLAT_COLOR:
        v_color = flat_color;
        break;
      case CLDVIS_HEIGHT:
        float range = bbox_maxp.y - bbox_minp.y;
        float height = position.y - bbox_minp.y;
        float hf = height/range;
        v_color = (1-hf)*vec4(0.0, 0.027, 0.698, 1.0) + hf*vec4(1.0, 0.824, 0.098, 1.0);
        break;
      case CLDVIS_NORMAL:
        v_color = vec4((vec3(model*vec4(normal, 0.0))+1.0) * 0.5,1.0);
        break;
      case CLDVIS_INSTANCE_ID:
        int type_id = int(user_data_a.g);
        v_color = cldvis_hex2color(CLDVIS_COLOR_TABLE[type_id % 40]);
        if( type_id == 1024 ) { v_color = cldvis_hex2color( 0xff00ff ); }
        break;
      case CLDVIS_CLASS_ID:
        int segment_id = int(user_data_a.b);
        v_color = cldvis_hex2color(CLDVIS_COLOR_TABLE[segment_id % 40]);
        if( segment_id == 0 ) { v_color = cldvis_hex2color( 0xff00ff ); }
        break;
      case CLDVIS_QUALITY:
        // TODO(maciej): Change this to divergent colors;
        vec4 col_a = vec4( 0.1, 0.6, 0.9, 1.0 );
        vec4 col_b = vec4( 0.8, 0.2, 0.1, 1.0 );
        v_color = quality*col_a + (1.0-quality)*col_b;
        break;
      case CLDVIS_UNIFORM_ID:
        v_color = cldvis_hex2color(CLDVIS_COLOR_TABLE[uniform_id%40]);
        if( uniform_id == 1024 ) { v_color = cldvis_hex2color( 0xff00ff ); }
        break;
      case CLDVIS_PHONG:
        float t = ndotl;
        vec3 c_a = vec3(0.76, 0.59, 0.44);
        vec3 c_b = vec3(0.38, 0.28, 0.21);
        vec3 c1 = t*c_a + (1.0-t)*c_b;
        vec3 c2 = 0.6 * pow(ndoth, 160.0) * vec3(0.95, 0.95, 0.8);
        vec3 c  = clamp(c1 + c2, 0, 1);
        v_color = vec4(c, 1.0);
        break;
      default:
        v_color = color_a;
    }  
  }
);

ctx->shader_source.surfel_gs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  layout( points ) in;
  layout( triangle_strip, max_vertices = 4 ) out;
  
  uniform mat4 mvp;
  uniform mat4 mv;
  uniform mat4 model;

  in vec4 v_color[];
  in vec3 v_normal[];
  flat in float v_radius[];
  flat in float v_point_depth[];
  out vec4 g_color;
  out vec3 g_normal;
  out vec2 g_tcoord;

  uniform float max_depth;
  uniform float point_size;

  void main()
  {
    if( v_point_depth[0] > max_depth ) return;
    vec4 p = gl_in[0].gl_Position; 
    vec3 n = v_normal[0];
    vec4 c = v_color[0];
    float radius = point_size * v_radius[0];
    float a = n.y;
    float b = n.x;
    vec4 t;
    if( abs(a) > 0.001) 
    {  
      t = vec4(normalize(vec3( a, -b, 0.0)), 0.0);
    }
    else
    {
      a = n.z;
      t = vec4(normalize(vec3( a, 0.0, -b)), 0.0);
    }
    vec4 s = vec4(normalize(cross(n, vec3(t))), 0.0);
    
    
    n = vec3(model * vec4(n, 0.0));

    gl_Position = mvp*(p - radius*t - radius*s);
    g_color = c;
    g_normal = n;
    g_tcoord = vec2(-1.0, -1.0);
    EmitVertex();

    gl_Position = mvp*(p + radius*t - radius*s);
    g_color = c;
    g_normal = n;
    g_tcoord = vec2(1.0, -1.0);
    EmitVertex();

    gl_Position = mvp*(p - radius*t + radius*s);
    g_color = c;
    g_normal = n;
    g_tcoord = vec2(-1.0, 1.0);
    EmitVertex();

    gl_Position = mvp*(p + radius*t + radius*s);
    g_color = c;
    g_normal = n;
    g_tcoord = vec2(1.0, 1.0);
    EmitVertex();

    EndPrimitive();
  }
);

ctx->shader_source.surfel_fs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  in vec4 g_color;
  in vec3 g_normal;
  in vec2 g_tcoord;
  out vec4 frag_color;
  void main()
  {
    float radius = sqrt(g_tcoord.x*g_tcoord.x + g_tcoord.y*g_tcoord.y);
    // if( g_color.r == 1.0 && g_color.g == 0.0f && g_color.b == 1.0f ) discard;
    if( radius > 1.0 ) discard;
    frag_color = g_color;
  }
);

/*
ctx->shader_source.splat_vis_fs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  in vec4 g_color;
  in vec3 g_normal;
  in vec2 g_tcoord;
  layout(location = 0) out vec4 frag_color;
  void main() {
    float radius = sqrt(g_tcoord.x*g_tcoord.x + g_tcoord.y*g_tcoord.y);
    if( radius > 1.0 ) discard;
    frag_color = radius * g_color;
    gl_FragDepth = gl_FragCoord.z + 0.0001f;//TODO(maciej): This seems to be causing troubles
  });

ctx->shader_source.splat_accum_fs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  in vec4 g_color;
  in vec3 g_normal;
  in vec2 g_tcoord;
  layout(location = 0) out vec4 frag_color;
  layout(location = 1) out vec4 frag_normal;
  layout(location = 2) out vec4 frag_weight;
  void main() {
    float radius = sqrt(g_tcoord.x*g_tcoord.x + g_tcoord.y*g_tcoord.y);
    if( radius > 1.0 ) discard;
    frag_color = (1.0 - radius) * g_color;
    frag_normal = vec4(g_normal, 1.0);
    frag_weight = vec4(1.0-radius);
  });
*/

ctx->shader_source.normal_gs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  layout( points ) in;
  layout( line_strip, max_vertices = 2 ) out;
  
  uniform mat4 mvp;

  in vec4 v_color[];
  in vec3 v_normal[];
  out vec4 g_color;
  // out vec3 g_normal;

  uniform float normal_length;

  void main()
  {
    vec4 p = gl_in[0].gl_Position; 
    vec3 n = v_normal[0];
    vec4 c = v_color[0];

    gl_Position = mvp * p;
    g_color = c;
    // g_normal = n;
    EmitVertex();

    gl_Position = mvp * (p+0.01f*normal_length*vec4(n, 0.0));
    g_color = c;
    // g_normal = n;
    EmitVertex();

    EndPrimitive();
  }
);

ctx->shader_source.normal_fs = (char *)MSHGFX_SHADER_HEAD MSHGFX_SHADER_STRINGIFY(
  in vec4 g_color;
  out vec4 frag_color;
  void main() {
    frag_color = g_color;
  });
#undef CLDVIS_SHADER_CONSTANTS
}

#endif /*CLOUDVIS_IMPLEMENTATION*/