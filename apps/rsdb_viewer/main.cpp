/*
NOTES: For now this is a tool which creates window for visual debugging. However,
       once it works, the functionality might be stripped.
*/
// #define MSH_STD_INCLUDE_LIBC_HEADERS
#define MSH_STD_IMPLEMENTATION
#define MSH_ARGPARSE_IMPLEMENTATION
#define MSH_VEC_MATH_IMPLEMENTATION
#define MSH_GFX_IMPLEMENTATION
#define MSH_GEOMETRY_IMPLEMENTATION
#define MSH_CAM_IMPLEMENTATION
#define MSH_PLY_IMPLEMENTATION
#define MSH_HASH_GRID_IMPLEMENTATION
#define RS_DATABASE_IMPLEMENTATION
#define RS_POINTCLOUD_IMPLEMENTATION
#define FILEPATH_HELPERS_IMPLEMENTATION
#define HASHTABLE_IMPLEMENTATION

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_IMPLEMENTATION
#define NK_GLFW_GL3_IMPLEMENTATION
#define GLFW_INCLUDE_NONE
#define CLOUDVIS_USE_NK_GUI
#define CLOUDVIS_IMPLEMENTATION
#define DEBUGVIS_USE_NK_GUI
#define DEBUGVIS_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdarg>
#include <cstddef>
#include <cstdbool>
#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cctype>

#include "msh/msh_std.h"
#include "msh/msh_argparse.h"
#include "msh/msh_vec_math.h"
#include "msh/msh_geometry.h"
#include "msh/msh_ply.h"
#include "msh/msh_hash_grid.h"
#include "mg/hashtable.h"

#include "filepath_helpers.h"
#include "rs_pointcloud.h"
#include "rs_database.h"

#if !defined(MSH_NO_WINDOW)
#include "msh/msh_gfx.h"
#include "msh/msh_cam.h"
#include "nuklear/nuklear.h"
#include "GLFW/glfw3.h"
#include "nuklear/nuklear_glfw_gl3.h"
#include "im3d.h"
#include "im3d_math.h" // TODO(maciej): this is optional, I don't need this.
#include "cloudvis.h"
#include "debugvis.h"
#include "stb/stb_image_write.h"
#endif

// Let's ask compiler nicely to generate code for these functions:
template int* msh_array__grow<int>(int* arr, unsigned long long new_len, unsigned long long elem_size );
template rs_object_placement* msh_array__grow<rs_object_placement>(rs_object_placement* arr, unsigned long long new_len, unsigned long long elem_size );
template vec3f* msh_array__grow<vec3f>(vec3f* arr, unsigned long long new_len, unsigned long long elem_size );

///////////////////////////////////////////////////////////////////////////////////
// Options
///////////////////////////////////////////////////////////////////////////////////

static const char* PROGRAM_NAME = "rsdb_viewer";

enum db_vis_type_e { DBV_PROPOSALS, DBV_OPTIMIZED, DBV_IDENTITY};

typedef struct screenshot_options
{
  int16_t width;
  int16_t height;
  int16_t show_scan;
  int16_t show_shapes;
  char* filename; 
 
  msh_vec3_t eye;
  msh_vec3_t center;
  msh_vec3_t up;

  msh_vec4_t col_a;
  msh_vec4_t col_b;

  enum db_vis_type_e vis_type;
  int16_t show_surfel;
  int16_t show_edl;
  int16_t pointcloud_res;
  int16_t coloring_mode;
} screenshot_opts_t;

typedef struct options
{
  char* input_database_filename;
  bool print_verbose;

  screenshot_opts_t s_options;
} options_t;


////////////////////////////////////////////////////////////////////////////////////////////////////
// RENDERING
////////////////////////////////////////////////////////////////////////////////////////////////////

mshgfx_geometry_t
convert_tsdf_to_gpu( float* tsdf, int x_res, int y_res, int z_res, msh_bbox_t bbox, float max_dist )
{
  int n = x_res*y_res*z_res;
  float *pos          = (float*)malloc( n * 3 * sizeof(float) );
  float *nor          = (float*)malloc( n * 3 * sizeof(float) );
  uint8_t *col_a      = (uint8_t*)malloc( n * 4 );
  float *user_a       = (float *)malloc( n * 4 * sizeof(float) );

  float voxel_spacing = mshgeo_bbox_width(&bbox) / x_res;
  int idx             = 0;  
  int tsdf_idx        = 0;

  for(int x = 0; x < x_res ; ++x)
  {
    for(int y = 0; y < y_res ; ++y)
    {
      for(int z = 0; z < z_res ; ++z)
      {  
    
        float dist = tsdf[tsdf_idx++];
        if(dist < 0 ) continue;

        pos[3*idx+0] = bbox.min_p.x + (x+0.5)*voxel_spacing;
        pos[3*idx+1] = bbox.min_p.y + (y+0.5)*voxel_spacing;
        pos[3*idx+2] = bbox.min_p.z + (z+0.5)*voxel_spacing;
        
        nor[3*idx+0] = bbox.min_p.x + (x+0.5)*voxel_spacing;
        nor[3*idx+1] = bbox.min_p.y + (y+0.5)*voxel_spacing;
        nor[3*idx+2] = bbox.min_p.z + (z+0.5)*voxel_spacing;
          
        
        col_a[4*idx+0] = (uint8_t)(((1.0 - dist/max_dist)*0.0 + dist/max_dist*1.0) * 255.0f);
        col_a[4*idx+1] = (uint8_t)(((1.0 - dist/max_dist)*0.0 + dist/max_dist*1.0) * 255.0f);
        col_a[4*idx+2] = (uint8_t)(((1.0 - dist/max_dist)*0.0 + dist/max_dist*1.0) * 255.0f);
        col_a[4*idx+3] = 255;
      
        user_a[4*idx+0] = voxel_spacing*0.5;
        user_a[4*idx+1] = voxel_spacing*0.5;
        user_a[4*idx+2] = voxel_spacing*0.5;
        user_a[4*idx+3] = voxel_spacing*0.5;

        idx++;
      }
    }
  }
  mshgfx_geometry_data_t tsdf_data;
  tsdf_data.positions   = pos;
  tsdf_data.normals     = nor;
  tsdf_data.colors_a    = col_a;
  tsdf_data.user_data_a = user_a;
  tsdf_data.n_vertices  = idx;
  int flags =  MSHGFX_POSITION |MSHGFX_NORMAL | MSHGFX_COLOR_A | MSHGFX_USER_DATA_A;
  mshgfx_geometry_t tsdf_gpu;
  mshgfx_geometry_init(&tsdf_gpu, &tsdf_data, flags, MSHGFX_STATIC_DRAW );
  free(pos);
  free(nor);
  free(col_a);
  free(user_a);
  return tsdf_gpu;
}


mshgfx_geometry_t
convert_rs_pointcloud_to_gpu( rs_pointcloud_t* pc, int level )
{
  // allocate data
  int n          = pc->n_pts[level];
  float *pos     = (float*)pc->positions[level];
  float *nor     = (float*)pc->normals[level];
  uint8_t *col_a = (uint8_t*)malloc( n * 4 );
  float *user_a  = (float *)malloc( n * 4 * sizeof(float) );

  // convert data
  int idx = 0;
  for( int i = 0; i < n ; ++i )
  {
    col_a[4*idx+0] = (uint8_t)(pc->colors[level][i].x*255.0f);
    col_a[4*idx+1] = (uint8_t)(pc->colors[level][i].y*255.0f);
    col_a[4*idx+2] = (uint8_t)(pc->colors[level][i].z*255.0f);
    col_a[4*idx+3] = 255;

    idx++;
  }

  idx = 0;

  for( int i = 0; i < n ; ++i )
  {  
    user_a[4*idx+0] = pc->radii[level][i];
    user_a[4*idx+1] = (float)pc->instance_ids[level][i];
    user_a[4*idx+2] = (float)pc->class_ids[level][i];
    user_a[4*idx+3] = pc->qualities[level][i];     

    idx++;
  }
  mshgfx_geometry_data_t pointcloud_data;
  pointcloud_data.positions   = pos;
  pointcloud_data.normals     = nor;
  pointcloud_data.colors_a    = col_a;
  pointcloud_data.user_data_a = user_a;
  pointcloud_data.n_vertices  = idx;

  int flags =  MSHGFX_POSITION | MSHGFX_NORMAL | MSHGFX_COLOR_A | MSHGFX_USER_DATA_A;
  mshgfx_geometry_t pointcloud;
  mshgfx_geometry_init(&pointcloud, &pointcloud_data, flags, MSHGFX_STATIC_DRAW );

  free(col_a);
  free(user_a);
  return pointcloud;
}

void save_screenshot_name( const char* filename, int w, int h)
{
  int32_t row_size = w*4*sizeof(uint8_t);
  uint8_t* img_data = (uint8_t*)malloc(h*row_size);
  uint8_t* tmp_row  = (uint8_t*)malloc(row_size); 
  glReadPixels(	0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, img_data);

  uint8_t* source = img_data;
  uint8_t* dst = img_data+(h-1)*row_size;
  for( int i = 0; i < h / 2; ++i )
  {
    memcpy(tmp_row, source, row_size);
    memcpy(source, dst, row_size);
    memcpy(dst, tmp_row, row_size);
    source += row_size;
    dst -= row_size;
  }
  stbi_write_png(filename, w, h, 4, (void*)img_data, 0);
  free(img_data);
  free(tmp_row);
}

void save_screenshot_count(int w, int h)
{
  static int screenshot_count = 0;
  char buf[1024];
  snprintf(buf, 1024, "screenshot_%03d.png", screenshot_count++);
  
  save_screenshot_name(buf, w, h);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// NK helpers
////////////////////////////////////////////////////////////////////////////////////////////////////
int nk_init_context(struct nk_context** ctx, GLFWwindow* window)
{
  *ctx = nk_glfw3_init(window, NK_GLFW3_INSTALL_CALLBACKS);
  if(!*ctx) { return 0; }
  {
    struct nk_font_atlas *atlas;
    nk_glfw3_font_stash_begin(&atlas);
    nk_glfw3_font_stash_end();
  }
  return 1;
}


void 
nk_update_camera( struct nk_context* nk_ctx, msh_camera_t* cam, msh_vec4_t viewport, float fovy )
{
  struct nk_input input = nk_ctx->input;
  
  if( !nk_item_is_any_active(nk_ctx) && Im3d::GetActiveId() == Im3d::Id_Invalid )
  {
    msh_arcball_camera_update(cam,  
                              msh_vec2(input.mouse.prev.x,input.mouse.prev.y), 
                              msh_vec2(input.mouse.pos.x,input.mouse.pos.y),
                              input.mouse.buttons[NK_BUTTON_LEFT].down, 
                              input.mouse.buttons[NK_BUTTON_RIGHT].down,
                              0.1f*input.mouse.scroll_delta.y,  
                              viewport );
  }


  msh_camera_update_perspective(cam, fovy, viewport.z/viewport.w,
                                0.1f, 500.0f);
}

void nk_fps_counter(struct nk_context *ctx, msh_vec4_t viewport)
{
  nk_style_push_style_item(ctx, &ctx->style.window.fixed_background, 
                                nk_style_item_color(nk_rgba(0, 0, 0, 0)));
  if(nk_begin(ctx, "Timer", nk_rect(viewport.z - 100, viewport.w - 36, 80, 24),
           NK_WINDOW_BACKGROUND|NK_WINDOW_NO_INPUT|
           NK_WINDOW_NOT_INTERACTIVE|NK_WINDOW_NO_SCROLLBAR))
  {
    nk_layout_row_dynamic(ctx, 0, 1);
    char time_str[128];
    snprintf(time_str, 128, "%5.3f ms", nk_glfw3_get_delta_time());
    nk_label_colored( ctx, time_str, NK_TEXT_ALIGN_LEFT, 
                                                      nk_rgba(30, 30, 30, 255));
  }
  nk_end(ctx);
  nk_style_pop_style_item(ctx);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// GLFW HELPERS
////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct keyboard_state
{
  int pressed[512];
  bool ctrl_down;
} keyboard_state_t;

static keyboard_state_t keyboard;

void
glfw_keyboard_callback( GLFWwindow *window, int key, int scancode, int action, int mods)
{
  keyboard.pressed[ key ] = 0;
  // printf("%d | %d %d | %d \n", action, key, GLFW_KEY_LEFT_CONTROL, mods );
  if( action == GLFW_PRESS )
  {
    keyboard.pressed[ key ] = 1;
    if( key == GLFW_KEY_LEFT_CONTROL ) { keyboard.ctrl_down = 1; }
  }

  if( action == GLFW_RELEASE )
  {
    if( key == GLFW_KEY_LEFT_CONTROL ) { keyboard.ctrl_down = 0; }
  }

}

int 
glfw_init_window(GLFWwindow** window, int width, int height) 
{
  if( !glfwInit() ) return 0;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 1);

  *window = glfwCreateWindow(width, height, PROGRAM_NAME, NULL, NULL);
  if(!*window)
  {
    glfwTerminate();
    return 0;
  }

  glfwSetKeyCallback(*window, glfw_keyboard_callback);
  glfwMakeContextCurrent(*window);
  gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
  glfwSwapInterval(1);
  return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Keyboard shortcuts
////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t reset_cam = 0;
static uint8_t show_gui = 1;
static uint8_t take_screenshot = 0;

void
process_keyboard_shortcuts(cldvis_ctx_t* cldvis_ctx, dbgvis_ctx_t* dbgvis_ctx)
{
  cldvis_keyboard_shortcuts(cldvis_ctx, keyboard.pressed, keyboard.ctrl_down);
  dbgvis_keyboard_shortcuts(dbgvis_ctx, keyboard.pressed, keyboard.ctrl_down);
  if( keyboard.pressed[320]/*NUM_0*/ && !keyboard.ctrl_down ) 
  {
    reset_cam = 1;
    keyboard.pressed[320] = 0; 
  }
  
  if( keyboard.pressed['G'] && !keyboard.ctrl_down ) 
  {
    show_gui = !show_gui;
    keyboard.pressed['G'] = 0; 
  }

  if( keyboard.pressed['S'] && !keyboard.ctrl_down )
  {
    take_screenshot = 1;
    keyboard.pressed['S'] = 0; 
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// TMP I/O
////////////////////////////////////////////////////////////////////////////////////////////////////

void load_pose_proposals( const char* filename, 
                          msh_array(msh_array(msh_mat4_t))* poses,
                          msh_array(msh_array(float))* scores )
{
  int verbose = 0;
  if( !filename ) return;

  int retval = 0;
  FILE* fp = fopen(filename, "rb");
  
  if(!fp) { return; }
  int n_pose_arrays = -1;
  retval = fread(&n_pose_arrays, sizeof(int), 1, fp);
  int* pose_counts = (int*)malloc(n_pose_arrays*sizeof(int));
  for( int i = 0; i < n_pose_arrays; ++i )
  {
    int n_poses = -1;
    retval = fread(&n_poses, sizeof(int), 1, fp);
    if( retval != 1 ) { printf("Issue reading pose proposals!\n"); return; }
    pose_counts[i] = n_poses;
  }
  msh_cprintf( verbose, "Reading %d pose arrays!\n", n_pose_arrays );
  (*poses) = NULL;
  for(int i = 0 ; i < n_pose_arrays; ++i)
  {
    msh_array(msh_mat4_t) cur_poses = NULL;
    msh_array(float) cur_scores = NULL;
    int n_poses = pose_counts[i];
    if( n_poses )
    {
      float* pose_storage = (float*)malloc(n_poses * 17 * sizeof(float));
      float* pose_storage_ptr = pose_storage;
      retval = fread(pose_storage, sizeof(float)*17, n_poses, fp);
      if( retval != n_poses ) { printf("Issue reading pose proposals!\n"); return; }
      for( int j = 0; j < n_poses; ++j )
      {
        msh_mat4_t* pose = (msh_mat4_t*)pose_storage_ptr;
        msh_array_push(cur_poses, *pose);
        pose_storage_ptr += 16;
        float* score = (float*)pose_storage_ptr;
        msh_array_push(cur_scores, *score );
        pose_storage_ptr += 1;
      }
      free(pose_storage);
    }
    msh_array_push((*poses), cur_poses);
    msh_array_push((*scores), cur_scores);
  }
  free(pose_counts);
  fclose(fp);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Entry point
////////////////////////////////////////////////////////////////////////////////////////////////////

int
parse_arguments(int argc, char** argv, options_t *opts )
{
  // Defaults
  opts->input_database_filename        = NULL;
  opts->print_verbose                  = 0;

  opts->s_options.filename       = 0;
  opts->s_options.width          = 640;
  opts->s_options.height         = 480;
  opts->s_options.eye            = msh_vec3(0.0f, 10.0f, 0.0f);
  opts->s_options.center         = msh_vec3(0.0f,  0.0f, 0.0f);
  opts->s_options.up             = msh_vec3(0.0f,  1.0f, 1.0f);
  opts->s_options.col_a          = msh_vec4(0.0f,  0.0f, 0.0f, 0.0f);
  opts->s_options.col_b          = msh_vec4(0.0f,  0.0f, 0.0f, 0.0f);
  opts->s_options.vis_type       = DBV_OPTIMIZED;
  opts->s_options.show_surfel    = 1;
  opts->s_options.show_edl       = 1;
  opts->s_options.pointcloud_res = 0;
  opts->s_options.show_scan      = 1;
  opts->s_options.show_shapes    = 1;
  opts->s_options.coloring_mode  = 0;


  // Command line
  msh_argparse_t parser;
  msh_ap_init( &parser, PROGRAM_NAME, 
                    "rsdb file opengl viewer");
  msh_ap_add_string_argument(&parser, "input_database_filename", NULL, "Path to database from" 
                          "previous timestep", &opts->input_database_filename, 1 );
  msh_ap_add_bool_argument(&parser, "--verbose", "-v", "Print verbose information",
                        &opts->print_verbose, 0 );
  msh_ap_add_string_argument(&parser, "--screenshot_filename", "-i", "Name of the screenshot",
                          &opts->s_options.filename, 1 );
  msh_ap_add_short_argument(&parser, "--screenshot_resolution", "-r", "Resolution of the screenshot",
                          &opts->s_options.width, 2 );
  msh_ap_add_float_argument(&parser, "--camera_look_at", "-c", "Eye/Center/Up vector triplet for look-at "
                         "matrix", &opts->s_options.eye.x, 9 );       
  msh_ap_add_short_argument( &parser, "--coloring_mode", "-m", "A value for different coloring modes", 
                             &opts->s_options.coloring_mode, 1 );

  return msh_ap_parse( &parser, argc, argv );
}


// Helpers
void
create_arrangement_filename( char (*filename)[1024], rs_scene_t* scene )
{
  char scene_name[1024];
  char timestamp_name[1024];
  char* pch = strchr(scene->scn_filename, '/');
  size_t scene_n_chars = pch - scene->scn_filename;
  strncpy( scene_name, scene->scn_filename, scene_n_chars );
  scene_name[scene_n_chars] = 0;
  char* substr = scene->scn_filename+scene_n_chars+1; 
  pch = strchr(substr, '/');
  size_t timestamp_n_chars = pch - (substr);
  strncpy(timestamp_name, substr, timestamp_n_chars);
  timestamp_name[timestamp_n_chars] = 0;

  sprintf(*filename, "%s/arrangements/%s", scene_name, timestamp_name);
  strcat((*filename), ".bin");
}

int 
main( int argc, char** argv )
{
  uint64_t st, et;
  options_t opts;
  if( !parse_arguments(argc, argv, &opts ) ) { exit( EXIT_FAILURE ); }
  
  // Get the info from database (note: we don't use database model loading here)
  rsdb_t* rsdb = rsdb_init();

  msh_cprintf(opts.print_verbose, "IO: Loading the database...\n");
  st = msh_time_now();
  rsdb_load( rsdb, opts.input_database_filename, 1 );
  et = msh_time_now();

  int n_objects = (int)msh_array_len(rsdb->objects);
  int n_scenes = msh_array_len(rsdb->scenes);
  int MIN_IDX = 0;
  int MAX_IDX = n_scenes;

  msh_cprintf(opts.print_verbose, "IO: Load .rsdb file %s with %d objects, %d scenes and %zu arrangements in %f ms.\n", 
                                    opts.input_database_filename, n_objects, n_scenes,
                                    msh_array_len( rsdb->arrangements ), 
                                    msh_time_diff_ms( et, st ) );

  // Helper data structures
  msh_array( rs_pointcloud_t* ) shapes = {0};
  msh_array( rs_pointcloud_t* ) input_scenes = {0};

  for( int i = 0; i < n_objects; ++i )
  {
    msh_array_push( shapes, rsdb->objects[i].shape );
  }

  for( int i = 0; i < n_scenes; ++i )
  {
    msh_array_push( shapes, rsdb->scenes[i].shape );
    msh_array_push( input_scenes, rsdb->scenes[i].shape );
  }

  // Pose proposal
  for( int i = MIN_IDX; i < MAX_IDX; ++i)
  {
    if( rsdb->scenes[i].pose_proposal_filename )
    {
      load_pose_proposals( rsdb->scenes[i].pose_proposal_filename, 
                           &rsdb->proposed_poses[i], &rsdb->proposed_poses_scores[i] );
    }
  }
  // add empty arrangement for novel scenes
  if( msh_array_len(rsdb->scenes) > msh_array_len(rsdb->arrangements) )
  {
    int n_added = 0;
    while( msh_array_len(rsdb->arrangements) < msh_array_len(rsdb->scenes) )
    {
      msh_array(rs_obj_plcmnt_t) arrangement = {0};
      msh_array_push( rsdb->arrangements, arrangement );
      n_added++;
    }
    msh_cprintf(opts.print_verbose, "IO: Added %d empty arrangements!\n", n_added);
  }

  // novel scene saliency computation
  int TIME_IDX = msh_array_len(rsdb->arrangements) - 1;

  rs_pointcloud_compute_levels( rsdb->scenes[TIME_IDX].shape );

////////////////////////////////////////////////////////////////////////////////////////////////////
// VISUALIZATION
////////////////////////////////////////////////////////////////////////////////////////////////////
  msh_camera_t camera = {0};
  cldvis_ctx_t cldvis_ctx = {0};
  dbgvis_ctx_t dbgvis_ctx = {0};
  struct nk_context* nk_ctx = NULL;
  GLFWwindow* window = NULL;
  int n_shapes = n_objects + n_scenes;
  
  float x_scale = 2.75;
  float y_scale = 1.75;
  int init_win_w = (int)(640 * x_scale);
  int init_win_h = (int)(480 * y_scale);
  if( opts.s_options.filename )
  {
    init_win_w = opts.s_options.width;
    init_win_h = opts.s_options.height;
  }
    
  if( !glfw_init_window(&window, init_win_w, init_win_h))
  {
    msh_eprintf("Could not initialize window!\n");
    exit(EXIT_FAILURE);
  }
  
  if(!cldvis_init_context(&cldvis_ctx))
  {
    printf("Could not initialize rendering ctx!\n");
    exit(EXIT_FAILURE);
  }
  cldvis_ctx.display_opts.surfel_size = 1.5f;
  cldvis_ctx.display_opts.max_depth   = 10.0f;
  cldvis_ctx.display_opts.fovy        = 2.0f * 0.404891786;
  
  if(!dbgvis_init_context(&dbgvis_ctx))
  {
    printf("Could not initialize debug rendering ctx!\n");
    exit(EXIT_FAILURE);
  }
  dbgvis_ctx.display_opts.show_axis   = 1;
  dbgvis_ctx.display_opts.show_grid   = 1;

  mshgfx_geometry_t *pointclouds[RSPC_N_LEVELS] = {NULL};
  msh_array(msh_mat4_t) poses                   =  NULL;
  msh_array(msh_bbox_t) bboxes                  =  NULL;
  msh_array(int8_t) visibility                  =  NULL;
  msh_array(int32_t) unique_ids                 =  NULL;
  msh_array(int32_t) active_pose_ind            =  NULL;
 
  //Upload database to gpu
  for( int i = 0; i < n_shapes; ++i )
  {
    for( int j = 0; j < RSPC_N_LEVELS; ++j )
    {
      mshgfx_geometry_t pc = convert_rs_pointcloud_to_gpu( shapes[i], j );
      msh_array_push( pointclouds[j], pc );
    }
    msh_array_push(bboxes, shapes[i]->bbox);
    msh_array_push(visibility, 1);
    msh_array_push(unique_ids, i);
    msh_array_push(active_pose_ind, 0);

    msh_mat4_t pose = msh_mat4_identity();
    msh_array_push(poses, pose);
  }
  
  // Upload names for visualization
  msh_array(char*) database_contents_names = NULL;
  for( int i = 0 ; i < n_objects; ++i)
  {
    rs_object_t* object = &rsdb->objects[i];
    char* name = (char*)hashtable_find( &rsdb->idx_to_class_name, (uint64_t)object->class_idx );
    char buf[512];
    sprintf(&buf[0], "%s %02d|%02d: %s", (object->is_shape_prior ? "SPr" : "Obj"),
                                     i, object->uidx, name );
    msh_array_push(database_contents_names, msh_strdup(buf));
  }
  for( int i = 0; i < n_scenes; ++i )
  {
    msh_array_push(database_contents_names, rsdb->scenes[i].scn_filename );
  }

  for( int i = 0; i < n_scenes; ++i )
  {
    msh_array_push(database_contents_names, rsdb->scenes[i].scn_filename );
  }

  msh_camera_init( &camera, msh_vec3(5.0, 5.0, 5.0), msh_vec3(0.0, 0.0, 0.0),
                            msh_vec3(0.0, 1.0, 0.0), 0.75, 1.0, 0.1, 500.0 );

  if( opts.s_options.filename )
  {
    msh_camera_init( &camera, opts.s_options.eye, opts.s_options.center,
                              opts.s_options.up, 0.75, 1.0, 0.1, 500.0 );
  }

  msh_vec4_t col_a = msh_vec4( 0.2f, 0.2f, 0.2f, 1.0f );
  msh_vec4_t col_b = msh_vec4( 0.5f, 0.5f, 0.5f, 1.0f );
  if( opts.s_options.filename )
  {
    col_a = opts.s_options.col_a;
    col_b = opts.s_options.col_b;
  }
  msh_rand_ctx_t rand_gen;
  msh_rand_init( &rand_gen, 12346ULL );

  if( !nk_init_context(&nk_ctx, window) )
  {
    printf( "Could not initialize gui!\n" );
    exit( EXIT_FAILURE );
  }

  char test_buf[1024] = {0};
  while( !glfwWindowShouldClose( window ) )
  {

    static int32_t resolution_idx = 1;
    glfwPollEvents();
    static int32_t fb_w = 0, fb_h = 0;
    static int32_t win_w = 0, win_h = 0;
    int32_t prev_w = win_w, prev_h = win_h;
    glfwGetWindowSize( window, &win_w, &win_h );
    glfwGetFramebufferSize( window, &fb_w, &fb_h );
    msh_vec4_t viewport = msh_vec4( 0.0f, 0.0f, (float)fb_w, (float)fb_h );
    if( prev_w != win_w || prev_h != win_h )
    {
      cldvis_resize( &cldvis_ctx, viewport );
    }

    static int32_t db_vis_type     = DBV_OPTIMIZED; 
    int32_t new_db_vis_type        = db_vis_type;
    static int32_t edit_focused    = 0;
    static int32_t show_objects    = 1;
    static int32_t show_input_scan = 1;
    static int32_t show_gizmos     = 1;
    static int32_t time_change     = 0;
    static int32_t show_scene_grid = 0;
    static int32_t show_arrangement_grid = 0;
    static int32_t show_scene_coverage_grid = 0;
    static int32_t show_intersection_grid = 0;
    nk_glfw3_new_frame();

    if( show_gui )
    {
      nk_fps_counter( nk_ctx, viewport );
      if( nk_begin( nk_ctx, "GUI1", nk_rect( 5, 5, 225, 420 ),
          NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE ) ) 
      {
        nk_layout_row_dynamic( nk_ctx, 24, 1 );
        int32_t cur_time_idx = TIME_IDX;
        nk_property_int( nk_ctx, "Time", MIN_IDX, &cur_time_idx, MAX_IDX-1, 1, 1 );
        if( cur_time_idx != TIME_IDX )
        {
          TIME_IDX = cur_time_idx;
          time_change = 1;
        }
        
        if( show_input_scan )
        {
          for( int32_t i = 0 ; i < n_scenes ; ++i )
          {
            if( i != TIME_IDX ) 
            { 
              visibility[n_objects+i] = 0; 
            }
            if( time_change && i == TIME_IDX ) 
            { 
              visibility[n_objects+i] = 1; 
            }
          }
        }
        // Resolution controls
        nk_property_int( nk_ctx, "Resolution", 0, &resolution_idx, RSPC_N_LEVELS - 1, 1, 0.25 );
  
        static const float text_height = 18.0f;
        static const float widths[] = {0.8f, 0.2f};
        nk_layout_row( nk_ctx, NK_DYNAMIC, text_height, 2, widths );
        struct nk_style_button bstyle = nk_ctx->style.button;
        bstyle.padding.x = -1.0; bstyle.padding.y = -1.0;
        enum nk_symbol_type vis_symbol = NK_SYMBOL_CIRCLE_SOLID;
        
        // Visibility toggles
        vis_symbol = (show_objects) ? NK_SYMBOL_CIRCLE_SOLID : NK_SYMBOL_CIRCLE_OUTLINE;
        nk_label( nk_ctx, "Show Objects", NK_TEXT_LEFT );
        if( nk_button_symbol_styled( nk_ctx, &bstyle, vis_symbol ) )
        {
          show_objects = !show_objects;
          for( int i = 0; i < n_objects; ++i ) { visibility[i] = show_objects; }                   
        }
        
        vis_symbol = (show_input_scan) ? NK_SYMBOL_CIRCLE_SOLID : NK_SYMBOL_CIRCLE_OUTLINE;
        nk_label( nk_ctx, "Show Scans", NK_TEXT_LEFT );
        if( nk_button_symbol_styled(nk_ctx, &bstyle, vis_symbol ) )
        {
          show_input_scan = !show_input_scan;
          for( int i = n_objects; i < n_objects + n_scenes; ++i ) { visibility[i] = show_input_scan; }
        }

        // Pose display options
         nk_layout_row_dynamic( nk_ctx, text_height, 1 );
        static const char *pose_labels[] = {"Proposals", "Optimized", "Identity" };
        new_db_vis_type = nk_combo( nk_ctx, pose_labels, msh_count_of( pose_labels ), 
                                    db_vis_type, (int)text_height, nk_vec2( 175, 175 ) );

        // Fine grain visibility controls
        if( nk_tree_push( nk_ctx, NK_TREE_TAB, "RSDB Contents", NK_MINIMIZED ) ) 
        {
          const float ratio[] = {0.6f, 0.1f, 0.3f};
          nk_layout_row( nk_ctx, NK_DYNAMIC, text_height, 3, ratio );

          for( int i = 0; i < n_objects; ++i )
          {
            nk_label( nk_ctx, database_contents_names[i], NK_TEXT_LEFT );
            vis_symbol = (visibility[i]) ? NK_SYMBOL_CIRCLE_SOLID : NK_SYMBOL_CIRCLE_OUTLINE;

            if( nk_button_symbol_styled( nk_ctx, &bstyle, vis_symbol ) )
            {
              visibility[i] = !visibility[i];                    
            }

            if( new_db_vis_type == DBV_PROPOSALS ) 
            {
              if( msh_array_len( rsdb->proposed_poses[TIME_IDX] ) && rsdb->proposed_poses[TIME_IDX][i] )
              {
                int max_idx = (int)msh_array_len( rsdb->proposed_poses[TIME_IDX][i] ) - 1;
                nk_property_int( nk_ctx, "#id", -1, &active_pose_ind[i], max_idx, 1, 0.25 );
                if( active_pose_ind[i] >= 0 )
                {
                  poses[i] = rsdb->proposed_poses[TIME_IDX][i][active_pose_ind[i]];
                }
                else
                {
                  poses[i] = msh_mat4_identity();
                }
              } 
              else
              {
                nk_spacing( nk_ctx, 1 );
              }
            }
            else
            {
              nk_spacing( nk_ctx, 1 );
            }
            
          }
          nk_tree_pop( nk_ctx );
        }
      }
      nk_end( nk_ctx );  

      if( nk_begin( nk_ctx, "GUI2", nk_rect( win_w - 230.0f, 5.0f, 225.0f, 378.0f ),
                    NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_SCALABLE ) ) 
      {
        if( nk_tree_push( nk_ctx, NK_TREE_TAB, "Pointcloud", NK_MAXIMIZED ) ) 
        {
            cldvis_nk_gui( &cldvis_ctx, nk_ctx );
            nk_tree_pop( nk_ctx );
        }

        if( nk_tree_push( nk_ctx, NK_TREE_TAB, "Helpers", NK_MINIMIZED ) ) 
        {
            dbgvis_nk_gui(&dbgvis_ctx, nk_ctx);
            nk_layout_row_dynamic(nk_ctx, 24, 1);
            
            struct nk_colorf nk_col_a;
            nk_col_a.r = col_a.x;
            nk_col_a.g = col_a.y;
            nk_col_a.b = col_a.z;
            nk_col_a.a = col_a.w;

            struct nk_colorf nk_col_b;
            nk_col_b.r = col_b.x;
            nk_col_b.g = col_b.y;
            nk_col_b.b = col_b.z;
            nk_col_b.a = col_b.w;

            if (nk_combo_begin_color(nk_ctx, nk_rgb_cf(nk_col_a), nk_vec2(200,400))) {
                nk_layout_row_dynamic(nk_ctx, 120, 1);
                nk_col_a = nk_color_picker(nk_ctx, nk_col_a, NK_RGBA);

                nk_layout_row_dynamic(nk_ctx, 25, 1);
                nk_col_a.r = nk_propertyf(nk_ctx, "#R:", 0, nk_col_a.r, 1.0f, 0.01f, 0.005f);
                nk_col_a.g = nk_propertyf(nk_ctx, "#G:", 0, nk_col_a.g, 1.0f, 0.01f, 0.005f);
                nk_col_a.b = nk_propertyf(nk_ctx, "#B:", 0, nk_col_a.b, 1.0f, 0.01f, 0.005f);
                nk_col_a.a = nk_propertyf(nk_ctx, "#A:", 0, nk_col_a.a, 1.0f, 0.01f, 0.005f);
              
                nk_combo_end(nk_ctx);
            }
            col_a = msh_vec4(nk_col_a.r, nk_col_a.g, nk_col_a.b, nk_col_a.a);

            if (nk_combo_begin_color(nk_ctx, nk_rgb_cf(nk_col_b), nk_vec2(200,400))) {
                nk_layout_row_dynamic(nk_ctx, 120, 1);
                nk_col_b = nk_color_picker(nk_ctx, nk_col_b, NK_RGBA);

                nk_layout_row_dynamic(nk_ctx, 25, 1);
                nk_col_b.r = nk_propertyf(nk_ctx, "#R:", 0, nk_col_b.r, 1.0f, 0.01f, 0.005f);
                nk_col_b.g = nk_propertyf(nk_ctx, "#G:", 0, nk_col_b.g, 1.0f, 0.01f, 0.005f);
                nk_col_b.b = nk_propertyf(nk_ctx, "#B:", 0, nk_col_b.b, 1.0f, 0.01f, 0.005f);
                nk_col_b.a = nk_propertyf(nk_ctx, "#A:", 0, nk_col_b.a, 1.0f, 0.01f, 0.005f);
            
                nk_combo_end(nk_ctx);
            }
            col_b = msh_vec4(nk_col_b.r, nk_col_b.g, nk_col_b.b, nk_col_b.a);

            nk_tree_pop(nk_ctx);
        }
      }
      
      nk_ctx->current->bounds.x = win_w - 230.0f;
      nk_ctx->current->bounds.y = 5.0f;
      nk_end( nk_ctx );
    }

    // Select db visualization type
    if( new_db_vis_type != db_vis_type || time_change )
    {
      db_vis_type = new_db_vis_type;
      if(db_vis_type == DBV_PROPOSALS)
      {
        for( int i = 0; i < n_objects ; ++i ) 
        { 
          if( msh_array_len( rsdb->proposed_poses[TIME_IDX] ) && 
              msh_array_len( rsdb->proposed_poses[TIME_IDX][i] ) ) 
          { 
            poses[i] = rsdb->proposed_poses[TIME_IDX][i][0]; 
          }
          else
          { 
            visibility[i] = 0; 
          }
        }
      }
      else if( db_vis_type == DBV_IDENTITY )
      {
        for( int i = 0; i < n_objects; ++i )
        { 
          poses[i] = msh_mat4_identity();
        }
      }
    }
    
    // screenshot stuff?
    if( opts.s_options.filename )
    {
      if( opts.s_options.show_surfel ) cldvis_ctx.display_opts.shader_mode = CLDVIS_SURFEL;
      else                             cldvis_ctx.display_opts.shader_mode = CLDVIS_POINT;

      cldvis_ctx.display_opts.edl_use    = opts.s_options.show_edl;
      db_vis_type                        = opts.s_options.vis_type;
      cldvis_ctx.display_opts.color_mode = (cldvis_color_mode_t)opts.s_options.coloring_mode;
      resolution_idx                     = opts.s_options.pointcloud_res;
      
      for( int i = 0; i < n_objects; ++i ) { visibility[i] = opts.s_options.show_shapes; }                   
      for( int i = n_shapes-2; i < n_shapes; ++i ) { visibility[i] = opts.s_options.show_scan; }
    }

    if( !edit_focused ) { process_keyboard_shortcuts(&cldvis_ctx, &dbgvis_ctx); }

    if( reset_cam )
    {
      msh_camera_init( &camera, msh_vec3(2.5, 2.7, 1.0), msh_vec3(0.0, 0.0, 0.0),
                                msh_vec3(0.0, 1.0, 0.0), 0.75, 1.0, 0.1, 500.0 );
      reset_cam = 0;
    }
    nk_update_camera( nk_ctx, &camera, viewport, cldvis_ctx.display_opts.fovy );


    glEnable( GL_DEPTH_TEST );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glEnable( GL_CULL_FACE );

    mshgfx_background_gradient4fv(col_b, col_a);

    cldvis_new_frame( &cldvis_ctx, &camera );

    if( db_vis_type == DBV_PROPOSALS || db_vis_type == DBV_IDENTITY )
    {
      cldvis_render_clouds( &cldvis_ctx, 
                            pointclouds[resolution_idx], poses, bboxes, visibility, NULL,
                            msh_array_len( pointclouds[resolution_idx] ) );
    }
    else
    {
      rs_obj_plcmnt_t* arrangement = NULL;
      if( !msh_array_isempty(rsdb->arrangements) && db_vis_type == DBV_OPTIMIZED )
      {
        arrangement = rsdb->arrangements[TIME_IDX];
      }
      
      // Render scenes
      cldvis_render_clouds_range( &cldvis_ctx, pointclouds[resolution_idx], poses, 
                                  bboxes, visibility, unique_ids, n_objects, n_objects + n_scenes );

      // Render objects
      for( size_t i = 0 ; i < msh_array_len(arrangement); ++i )
      {
        rs_obj_plcmnt_t object_placement = arrangement[i];
        int segment_idx = object_placement.object_idx;
        int visible = visibility[segment_idx];
        msh_mat4_t* pose = &object_placement.pose;
        
        if( visible )
        {
            cldvis_render_cloud( &cldvis_ctx, &pointclouds[resolution_idx][segment_idx], 
                                 pose, object_placement.uidx, &bboxes[segment_idx] );
        }
      }
    }

    cldvis_end_frame( &cldvis_ctx ); 
    cldvis_present( &cldvis_ctx );

    dbgvis_new_frame( &dbgvis_ctx, &camera );
    msh_mat4_t axis_pose = msh_mat4_identity();
    dbgvis_render_axis( &dbgvis_ctx, axis_pose );
    dbgvis_render_grid( &dbgvis_ctx );
    dbgvis_end_frame( &dbgvis_ctx );


    if( opts.s_options.filename )
    {
      save_screenshot_name( opts.s_options.filename, fb_w, fb_h );
      break;
    }

    Im3d::Draw();

    if( take_screenshot )
    {
      save_screenshot_count( fb_w, fb_h );
      printf("SCREENSHOT\n");
      take_screenshot = 0;
    }


    nk_glfw3_render( NK_ANTI_ALIASING_ON, MAX_VERTEX_BUFFER, MAX_ELEMENT_BUFFER );

    glfwSwapBuffers( window );
  }

  cldvis_destroy_context( &cldvis_ctx );
  nk_glfw3_shutdown();

  glfwDestroyWindow( window );
  glfwTerminate();  

  return 0;
}