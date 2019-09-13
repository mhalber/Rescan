/*
NOTES: For now this is a tool which creates window for visual debugging. However,
       once it works, the functionality might be stripped.
*/
// #define MSH_STD_INCLUDE_LIBC_HEADERS
#define MSH_STD_IMPLEMENTATION
#define MSH_ARGPARSE_IMPLEMENTATION
#define MSH_VEC_MATH_IMPLEMENTATION
#define MSH_GFX_IMPLEMENTATION
#define MSH_CAM_IMPLEMENTATION
#define MSH_PLY_IMPLEMENTATION
#define MSH_HASH_GRID_IMPLEMENTATION
#define RS_DATABASE_IMPLEMENTATION
#define RS_POINTCLOUD_IMPLEMENTATION
#define RS_DISTANCE_FUNCTION_IMPLEMENTATION
#define FILEPATH_HELPERS_IMPLEMENTATION
#define HASHTABLE_IMPLEMENTATION

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
#include "icp.h"

#include "filepath_helpers.h"
#include "rs_pointcloud.h"
#include "rs_distance_function.h"
#include "rs_database.h"
#include "intersect.h"
#include "rs_pointcloud_filters.h"
#include "arrangement_optimization.h"
#include "database_update.h"
#include "diverging_map.inl"

// Let's ask compiler nicely to generate code for these functions:
template int32_t* msh_array__grow<int32_t>(int32_t* arr, unsigned long long new_len, unsigned long long elem_size );
template rs_object_placement* msh_array__grow<rs_object_placement>(rs_object_placement* arr, unsigned long long new_len, unsigned long long elem_size );
template rspf_plane_model* msh_array__grow<rspf_plane_model>(rspf_plane_model* arr, unsigned long long new_len, unsigned long long elem_size );
template rspf_edge* msh_array__grow<rspf_edge>(rspf_edge* arr, unsigned long long new_len, unsigned long long elem_size );
template vec3f* msh_array__grow<vec3f>(vec3f* arr, unsigned long long new_len, unsigned long long elem_size );

///////////////////////////////////////////////////////////////////////////////////
// Options
///////////////////////////////////////////////////////////////////////////////////

static const char* PROGRAM_NAME = "segment_transfer";

enum db_vis_type_e { DBV_PROPOSALS, DBV_OPTIMIZED, DBV_IDENTITY, DBV_SCENE_ALIGNMENT };


typedef struct options
{
  char* input_database_filename;
  char* output_database_filename;
  bool just_greedy_initialization;
  bool just_simulated_annealing;
  bool output_segmented_mesh;
  bool print_verbose;
} options_t;


////////////////////////////////////////////////////////////////////////////////////////////////////
// I/O
////////////////////////////////////////////////////////////////////////////////////////////////////

void 
save_arrangement( const char* filename, msh_array(rs_obj_plcmnt_t) arrangement )
{
  if( !filename ) return;
  
  FILE* fp = fopen(filename, "wb");
  if( fp )
  {
    int32_t n    = msh_array_len(arrangement);
    fwrite( &n, sizeof(int32_t), 1, fp );
    fwrite( &arrangement[0], sizeof(rs_obj_plcmnt_t), n, fp);
    fclose(fp);
  }
}

static rsdb_t* RSDB_SORT_PTR = NULL;
int32_t 
static_plcmnt_cmp( const void* a, const void* b )
{
  rs_obj_plcmnt_t* plcmnt_a = (rs_obj_plcmnt_t*)a;
  rs_obj_plcmnt_t* plcmnt_b = (rs_obj_plcmnt_t*)b;
  int32_t is_static_a = rsdb_is_object_static( RSDB_SORT_PTR, plcmnt_a->object_idx);
  int32_t is_static_b = rsdb_is_object_static( RSDB_SORT_PTR, plcmnt_b->object_idx);
  int32_t uidx_a = plcmnt_a->uidx;
  int32_t uidx_b = plcmnt_b->uidx;
  // sort first on whether is static or not and then invert sort on unique idx.
  int32_t cmp_a = is_static_b << 10 | uidx_a;
  int32_t cmp_b = is_static_a << 10 | uidx_b;
  return cmp_a - cmp_b;
}

void 
load_arrangement( const char* filename, msh_array(rs_obj_plcmnt_t) *arrangement )
{
  if( !filename ) return;
  
  FILE* fp = fopen(filename, "rb");
  if( fp )
  {
    int32_t n = -1;
    int32_t retval = 0;
    retval = fread( &n, sizeof(int32_t), 1, fp );
    if( retval != 1 ) { printf("Issue reading arrangement %s\n", filename ); return; }
    
    if( *arrangement == NULL ) { msh_array_fit((*arrangement), (size_t)n); }
    else 
    {
      msh_array_clear((*arrangement));
      msh_array_fit( (*arrangement), (size_t)n);
    }
    
    for( int32_t i = 0; i < n; ++i )
    {
      rs_obj_plcmnt_t placement = {0};
      retval = fread( &placement, sizeof(rs_obj_plcmnt_t), 1, fp);
      if( retval != 1 ) { printf("Issue reading arrangement %s\n", filename ); return; }
      msh_array_push(*arrangement, placement);
    }
    fclose(fp);
  }
}

void 
load_pose_proposals( const char* filename, msh_array(msh_array(msh_mat4_t))* poses, 
                     msh_array(msh_array(float))* scores )
{
  int32_t verbose = 0;
  if( !filename ) return;

  int32_t retval = 0;
  FILE* fp = fopen(filename, "rb");
  
  if(!fp) { return; }
  int32_t n_pose_arrays = -1;
  retval = fread(&n_pose_arrays, sizeof(int32_t), 1, fp);
  int32_t* pose_counts = (int32_t*)malloc(n_pose_arrays*sizeof(int32_t));
  for( int32_t i = 0; i < n_pose_arrays; ++i )
  {
    int32_t n_poses = -1;
    retval = fread(&n_poses, sizeof(int32_t), 1, fp);
    if( retval != 1 ) { printf("Issue reading pose proposals!\n"); return; }
    pose_counts[i] = n_poses;
  }
  msh_cprintf( verbose, "Reading %d pose arrays!\n", n_pose_arrays );
  (*poses) = NULL;
  for(int32_t i = 0 ; i < n_pose_arrays; ++i)
  {
    msh_array(msh_mat4_t) cur_poses = NULL;
    msh_array(float) cur_scores = NULL;
    int32_t n_poses = pose_counts[i];
    if( n_poses )
    {
      float* pose_storage = (float*)malloc(n_poses * 17 * sizeof(float));
      float* pose_storage_ptr = pose_storage;
      retval = fread(pose_storage, sizeof(float)*17, n_poses, fp);
      if( retval != n_poses ) { printf("Issue reading pose proposals!\n"); return; }
      for( int32_t j = 0; j < n_poses; ++j )
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

int32_t
parse_arguments(int32_t argc, char** argv, options_t *opts, rsao_opts_t* ao_opts )
{
  // Defaults
  opts->input_database_filename        = NULL;
  opts->output_database_filename       = NULL;
  opts->just_greedy_initialization     = 0;
  opts->just_simulated_annealing       = 0;
  opts->print_verbose                  = 0;
  opts->output_segmented_mesh          = 0;

  rsao_init_opts( ao_opts );
  
  // Command line
  msh_argparse_t parser;
  msh_ap_init( &parser, PROGRAM_NAME, 
                    "This program performs arrangement optimization and segmentation transfer between input object database and the input reconstruction (as a .ply file), producing segmentation of the input reconstruction");
  msh_ap_add_string_argument(&parser, "input_database_filename", NULL, "Path to database from" 
                          " previous timestep", &opts->input_database_filename, 1 );
  msh_ap_add_string_argument(&parser, "--output_database", "-o", "Path to databse .rsdb file estimated "
                          "arrangements.", &opts->output_database_filename, 1 );
  msh_ap_add_bool_argument(&parser, "--output_segmentation", "-s", "Enable writing of segmented meshes", 
                        &opts->output_segmented_mesh, 0 );
  msh_ap_add_bool_argument( &parser, "--just_simulated_annealing", NULL, "Optimize function using "
                            "simualted annealing only", &opts->just_simulated_annealing, 0 );
  msh_ap_add_bool_argument( &parser, "--just_greedy_initialization", NULL, "Optimize function using "
                            "greedy initialization only", &opts->just_greedy_initialization, 0 );
  msh_ap_add_bool_argument(&parser, "--verbose", "-v", "Print verbose information",
                        &opts->print_verbose, 0 );
  msh_ap_add_double_argument( &parser, "--efw_greedy", NULL, "Values for the weights on different energy terms during"                              " the greedy step(Scene Coverage, Geometry, Intersection, Hysteresis)",
                             &ao_opts->energy_function_weights_greedy[0], ET_N_TERMS );
  msh_ap_add_double_argument( &parser, "--efw_sa", NULL, "Values for the weights on different energy terms during "
                              " the Simulated Annealing step(Scene Coverage, Geometry, Intersection, Hysteresis)",
                             &ao_opts->energy_function_weights_sa[0], ET_N_TERMS );
  msh_ap_add_double_argument( &parser, "--likelihoods_sa", "-l", "Values for the likelihoods of simulated annealing"
                              " actions (Add, Remove, Replace, Swap, Move, Change Identity, Swap Identity)",
                             &ao_opts->simulated_annealing_action_likelihoods[0], RSAO_N_ACTIONS );
  msh_ap_add_int_argument( &parser, "--lower_index", NULL, "Index of starting sequence", &ao_opts->lower_idx, 1 );
  msh_ap_add_int_argument( &parser, "--upper_index", NULL, "Index of ending sequence", &ao_opts->upper_idx, 1 );
  msh_ap_add_int_argument( &parser, "--n_sa_iter", NULL, "Number of iterations for simulated annealing", 
                              &ao_opts->n_sa_iter, 1 );
  msh_ap_add_int_argument( &parser, "--n_past_steps", NULL, "Number of steps in the past we are allowed to look at",
                              &ao_opts->n_past_steps, 1 );
  return msh_ap_parse( &parser, argc, argv );
}

int32_t
main( int32_t argc, char** argv )
{
  uint64_t t1, t2;
  options_t opts;
  rsao_opts_t ao_opts = {0};
  if( !parse_arguments(argc, argv, &opts, &ao_opts) ) { exit( EXIT_FAILURE ); }
  
  // Get the info from database (note: we don't use database model loading here)
  rsdb_t* rsdb = rsdb_init();

  msh_cprintf(opts.print_verbose, "IO: Loading the database...\n");
  t1 = msh_time_now();
  rsdb_load( rsdb, opts.input_database_filename, 1 );
  t2 = msh_time_now();

  int32_t n_objects = (int32_t)msh_array_len(rsdb->objects);
  int32_t n_scenes = (int32_t)msh_array_len(rsdb->scenes);

  static int32_t MIN_IDX = ao_opts.lower_idx; 
  static int32_t MAX_IDX = ao_opts.upper_idx;
  MAX_IDX = msh_min(n_scenes, MAX_IDX);

  msh_cprintf(opts.print_verbose, "IO: Load .rsdb file %s with %d objects, %d scenes and %zu arrangements in %f ms.\n", 
                                    opts.input_database_filename, n_objects, n_scenes,
                                    msh_array_len( rsdb->arrangements ), 
                                    msh_time_diff_ms( t2, t1 ) );

  // Helper data structures
  msh_array( rs_pointcloud_t* ) shapes = {0};
  msh_array( rs_pointcloud_t* ) input_scenes = {0};

  for( int32_t i = 0; i < n_objects; ++i )
  {
    msh_array_push( shapes, rsdb->objects[i].shape );
  }

  for( int32_t i = 0; i < n_scenes; ++i )
  {
    msh_array_push( shapes, rsdb->scenes[i].shape );
    msh_array_push( input_scenes, rsdb->scenes[i].shape );
  }

  // Pose proposal
  for( int32_t i = MIN_IDX; i < MAX_IDX; ++i)
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
    int32_t n_added = 0;
    while( msh_array_len(rsdb->arrangements) < msh_array_len(rsdb->scenes) )
    {
      msh_array(rs_obj_plcmnt_t) arrangement = {0};
      msh_array_push( rsdb->arrangements, arrangement );
      n_added++;
    }
    msh_cprintf(opts.print_verbose, "IO: Added %d empty arrangements!\n", n_added);
  }

  int32_t TIME_IDX = msh_array_len(rsdb->arrangements) - 1;

  isect_grid3d_t scn_grd = {0};
  isect_grid3d_t arrangement_grd = {0};
  isect_grid3d_t isect_grd = {0};
  isect_grid3d_t saliency_grd = {0};
  msh_array( rspf_plane_model_t ) planes = {0};
  if( !msh_array_isempty( rsdb->scenes ) && 
      rsdb->scenes[TIME_IDX].pose_proposal_filename  )
  {
    // Initialize grids
    float voxel_size = 0.05f; 
    isect_grid3d_init( &scn_grd, &rsdb->scenes[TIME_IDX].shape->bbox, voxel_size );
    isect_grid3d_init( &arrangement_grd, &rsdb->scenes[TIME_IDX].shape->bbox, voxel_size );
    isect_grid3d_init( &isect_grd, &rsdb->scenes[TIME_IDX].shape->bbox, voxel_size );
    isect_grid3d_init( &saliency_grd, &rsdb->scenes[TIME_IDX].shape->bbox, 0.15f );
    ao_opts.scn_grd = &scn_grd;
    ao_opts.arrangement_grd = &arrangement_grd;
    ao_opts.isect_grd = &isect_grd;
    ao_opts.saliency_grd = &saliency_grd;

    // Compute scene saliency
    rspf_detect_planes( rsdb->scenes[TIME_IDX].shape, &planes );
    rspf_compute_plane_features( rsdb->scenes[TIME_IDX].shape, &planes );
    rspf_classify_planes( rsdb->scenes[TIME_IDX].shape, &planes );
    rsao_compute_scene_saliency( rsdb, TIME_IDX, &ao_opts );
    rs_pointcloud_compute_levels( rsdb->scenes[TIME_IDX].shape );
    rsao_rasterize_scene_to_grid( &rsdb->scenes[TIME_IDX], &scn_grd, 0.5f );

    ao_opts.n_past_steps = msh_min( msh_array_len(rsdb->arrangements) - 1, ao_opts.n_past_steps);
  }

  if( !opts.just_simulated_annealing )
  {
    t1 = msh_time_now();
    double score = rsao_compute_scene_alignment_score( rsdb, rsdb->arrangements[TIME_IDX], 
                                                      &ao_opts, ao_opts.energy_function_weights_greedy, 0 );
    double prev_score = 0.0;
    for(;;)
    {
      prev_score = score;
      score = rsao_greedy_step( rsdb, TIME_IDX, &ao_opts );
      if( prev_score - score > 0.000001 )
      {
        msh_array_pop( rsdb->arrangements[TIME_IDX] );
        break;
      }
      if( fabs( prev_score - score ) < 0.000001 ) { break; }
    };
    t2 = msh_time_now();
    msh_cprintf( 1, "ARRANGEMENT_OPTIMIZATION: Greedy estimation finished in %fs.\n", 
                      msh_time_diff_sec( t2, t1 ) );
  }

  if( !opts.just_greedy_initialization )
  {
    t1 = msh_time_now();
    rsao_simulated_annealing( rsdb, TIME_IDX, &ao_opts );
    t2 = msh_time_now();
    msh_cprintf( 1, "ARRANGEMENT_OPTIMIZATION: Optimization finished in %fs.\n", 
                      msh_time_diff_sec( t2, t1 ) );
  }

  t1 = msh_time_now();
  rsao_add_static_objects( rsdb, TIME_IDX );
  t2 = msh_time_now();
  msh_cprintf( 1, "LABEL_TRANSFER: Adding static objects finished in %fs.\n", 
                  msh_time_diff_sec( t2, t1 ) );

  t1 = msh_time_now();
  int32_t skip_static_objects = 1;
  rsdb_refine_alignment_of_objects_to_scene( rsdb, TIME_IDX, skip_static_objects );
  t2 = msh_time_now();
  msh_cprintf( 1, "ARRANGEMENT_OPTIMIZATION: Refining optimized poses done in %fs.\n", 
                    msh_time_diff_sec( t2, t1 ) );

  t1 = msh_time_now();
  rspf_arrangement_to_labels( rsdb, input_scenes[TIME_IDX], rsdb->arrangements[TIME_IDX], 0.05f, 0 );
  rspf_relabel_walls_and_floors( rsdb, input_scenes[TIME_IDX], &planes );
  rspf_smooth_labels( rsdb, input_scenes[TIME_IDX] ); 
  t2 = msh_time_now();
  msh_cprintf( 1, "LABEL_TRANSFER: Segmentation finished in %fs.\n", 
                            msh_time_diff_sec( t2, t1) );

  t1 = msh_time_now();
  rsdu_augment_database( rsdb, input_scenes[TIME_IDX], rsdb->arrangements[TIME_IDX] );
  t2 = msh_time_now();
  msh_cprintf( 1, "LABEL_TRANSFER: Database augmentation finished in %fs.\n", 
                    msh_time_diff_sec( t2, t1) );

  if( opts.output_database_filename )
  {
    t1 = msh_time_now();
    free(rsdb->scenes[TIME_IDX].scn_filename);
    free(rsdb->model_folder);
    rsdb->model_folder = create_model_folder_name( opts.output_database_filename );
    rsdb->scenes[TIME_IDX].scn_filename = create_output_segmentation_scene_filename( rsdb->model_folder );
    printf("%s %s\n", rsdb->model_folder, rsdb->scenes[TIME_IDX].scn_filename);
    rsdb_save( rsdb, opts.output_database_filename, 1 );
    rs_pointcloud_to_file( input_scenes[TIME_IDX], rsdb->scenes[TIME_IDX].scn_filename );
    rs_pointcloud__save_ply( rsdb->scenes[TIME_IDX].scn_filename, input_scenes[TIME_IDX], 1 );
    t2 = msh_time_now();
    msh_cprintf( 1, "IO: Saved database %s and segmented pointcloud %s in %fs.\n", 
                    opts.output_database_filename,
                    rsdb->scenes[TIME_IDX].scn_filename,
                    msh_time_diff_sec( t2, t1 ) );
  }

  return 0;
}
