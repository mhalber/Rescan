// TOOD(maciej): This version is done to create gt i guess... I just need it for one.
// TODO(maciej): Change the argument list to take just the class list.
// NOTE(maciej): this requires wierd database.rsdb file, only for class to idx table really. Should change it

#define MSH_STD_IMPLEMENTATION
#define MSH_PLY_IMPLEMENTATION
#define MSH_ARGPARSE_IMPLEMENTATION
#define MSH_VEC_MATH_IMPLEMENTATION
#define MSH_GEOMETRY_IMPLEMENTATION
#define MSH_HASH_GRID_IMPLEMENTATION
#define FILEPATH_HELPERS_IMPLEMENTATION
#define RS_POINTCLOUD_IMPLEMENTATION
#define RS_DATABASE_IMPLEMENTATION
#define HASHTABLE_IMPLEMENTATION
#define ICP_IMPLEMENTATION

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
#include "rs/icp.h"
#include "rs/filepath_helpers.h"
#include "rs/rs_pointcloud.h"
#include "rs/rs_database.h"

typedef struct options
{
  char* in_ply_filename;
  char* in_rsdb_filename;
  char* out_rsdb_filename;
  bool print_verbose;
} options_t;

void
pointcloud_to_rsdb( const rsdb_t* in_rsdb, rs_pointcloud_t* pointcloud, rsdb_t* out_rsdb, options_t *opts )
{
  // set initial values
  const int32_t lvl = 0;
  out_rsdb->version_major = in_rsdb->version_major;
  out_rsdb->version_minor = in_rsdb->version_minor;
  
  out_rsdb->model_folder  = create_model_folder_name( opts->out_rsdb_filename );
  
  // copy index-to-class and class-to-index tables
  int32_t itc_count = hashtable_count( &in_rsdb->idx_to_class_name );
  HASHTABLE_U64 const* itc_keys = hashtable_keys( &in_rsdb->idx_to_class_name );
  for( int32_t i = 0; i < itc_count; ++i )
  {
    char* name = (char*)hashtable_find( &in_rsdb->idx_to_class_name, itc_keys[i]);
    hashtable_insert( &out_rsdb->idx_to_class_name, itc_keys[i], name );
  }
  
  int32_t cti_count = hashtable_count( &in_rsdb->class_name_to_idx );
  HASHTABLE_U64 const* cti_keys = hashtable_keys( &in_rsdb->class_name_to_idx );
  int32_t* cti_items = (int32_t*)hashtable_items( &in_rsdb->class_name_to_idx );
  for( int32_t i = 0; i < cti_count; ++i )
  {
    hashtable_insert( &out_rsdb->class_name_to_idx, cti_keys[i], &cti_items[i] );
  }

  // add a new scene
  rs_scene_t scene = {};
  scene.uidx                   = msh_array_len( in_rsdb->scenes );
  scene.arrangement_idx        = scene.uidx;
  scene.scn_filename           = opts->in_ply_filename;
  scene.shape                  = pointcloud;
  msh_array_push( out_rsdb->scenes, scene );
  
  // determine what objects exist in the pointcloud
  hashtable_t found_ids = {};
  hashtable_init( &found_ids, sizeof( int32_t ), 512, NULL );
  for( size_t i = 0; i < pointcloud->n_pts[lvl]; ++i )
  {
    HASHTABLE_U64 instance_id = (HASHTABLE_U64)pointcloud->instance_ids[lvl][i];
    int32_t val = 0;
    int32_t* exists = (int32_t*)hashtable_find( &found_ids, instance_id ); 
    if( !exists) hashtable_insert( &found_ids, instance_id, &val );
  }
  printf("PC_TO_RSDB: Scene contains %d unique object instances\n", hashtable_count( &found_ids ) );

  // extract objects from pointcloud and add them to the arrangement
  int32_t all_ids = hashtable_count( &found_ids );
  HASHTABLE_U64 const* keys = hashtable_keys( &found_ids );
  msh_array(rs_obj_plcmnt_t) new_arrangement = {0};
  int32_t newly_added = 0;
  for( int32_t i = 0; i < all_ids; ++i )
  {
    printf( "PC_TO_RSDB: Working on object %2d/%2d...\n", i+1, all_ids );

    // create object placement
    rs_obj_plcmnt_t placement = {};
    placement.arrangement_idx = msh_array_len( in_rsdb->arrangements );
    placement.uidx            = keys[i];
    placement.pose_idx        = 0;
    placement.score           = 1.0f;
    placement.pose            = msh_mat4_identity();

    // create object (extract from new mesh)
    rs_object_t obj    = {};
    obj.uidx           = keys[i];
    obj.is_shape_prior = 0;
    obj.shape          = rs_pointcloud_copy_by_ids( pointcloud, 0, RS_PT_INSTANCE_ID, (int32_t*)&keys[i], 1, 0 );
    obj.class_idx      = obj.shape->class_ids[0][0];
    char* class_name   = rsdb_get_class_name( in_rsdb, obj.class_idx );
    printf("PC_TO_RSDB:    Class name: %15s | Class id: %5d | Instance id: %5d\n", class_name, obj.class_idx, obj.shape->instance_ids[0][0]);
   
    msh_vec3_t centroid = rs_pointcloud_centroid( obj.shape, 0 );
    centroid.y = 0;
    msh_mat4_t xform = msh_mat4_identity();
    if( !rsdb_is_class_static( in_rsdb, obj.class_idx ) )
    {
      xform = msh_translate( msh_mat4_identity(), msh_vec3_invert(centroid) );
      rs_pointcloud_transform( obj.shape, xform, 0 );
    }
    placement.pose = msh_mat4_inverse( xform );

    char output_name[512] = {0};
    sprintf( output_name, "%s.%03d.ply", class_name, obj.uidx );
    obj.filename = msh_strdup(output_name);
  
    newly_added += 1;
    
    msh_array_push( out_rsdb->objects, obj );
    msh_array_push( new_arrangement, placement );
  }
  hashtable_term( &found_ids );

  msh_array_push( out_rsdb->arrangements, new_arrangement );
  printf("PC_TO_RSDB: Added %d new objects\n", newly_added );

  // Set correct object ids
  for( size_t i = 0 ; i < msh_array_len(out_rsdb->arrangements); ++i )
  {
    msh_array(rs_obj_plcmnt_t) arrangement = out_rsdb->arrangements[i];
    for( size_t j = 0; j < msh_array_len(arrangement); ++j )
    {
      rs_obj_plcmnt_t *plcmnt = &arrangement[j];
      for( size_t k = 0; k < msh_array_len(out_rsdb->objects); ++k )
      {
        rs_object_t *obj = &out_rsdb->objects[k];
        if( plcmnt->uidx == obj->uidx ) 
        {
          plcmnt->object_idx = k; 
        }
      }
    }
  }
}

int32_t 
parse_arguments(int32_t argc, char** argv, options_t *opts )
{
  // Defaults
  opts->in_ply_filename   = NULL;
  opts->in_rsdb_filename  = NULL;
  opts->out_rsdb_filename = NULL;
  opts->print_verbose     = 0;
  
  // Command line
  msh_argparse_t parser;
  msh_ap_init( &parser, "seg2rsdb", 
               "This program generates rsdb file using a .ply file with segmentation fields" );
  msh_ap_add_string_argument( &parser, "ply_filename", NULL, "Path to the input .ply file",
                              &opts->in_ply_filename, 1 );
  msh_ap_add_string_argument( &parser, "class_to_idx", NULL, "Path to .txt file containing the class-to-index mapping",
                              &opts->in_rsdb_filename, 1 );
  msh_ap_add_string_argument( &parser, "out_rsdb_filename", NULL, "Path to output .rsdb",
                              &opts->out_rsdb_filename, 1 );
  msh_ap_add_bool_argument( &parser, "--verbose", "-v", "Print verbose information",
                            &opts->print_verbose, 0 );

  return msh_ap_parse( &parser, argc, argv );
}

int32_t main( int32_t argc, char** argv )
{
  uint64_t t1, t2;
  options_t opts;
  rsdb_t* in_rsdb = rsdb_init();
  rsdb_t* out_rsdb = rsdb_init();
  rs_pointcloud_t* pointcloud = rs_pointcloud_init( 1 );

  // Parse command line
  if( !parse_arguments(argc, argv, &opts) ) { exit( EXIT_FAILURE ); }
  
  // Parse pointcloud file
  t1 = msh_time_now();
  rs_pointcloud_from_file( pointcloud, opts.in_ply_filename );
  t2 = msh_time_now();
  msh_cprintf( opts.print_verbose, "IO: Reading a ply file %s with %d points in %fs.\n", 
               opts.in_ply_filename, (int32_t)pointcloud->n_pts[0], msh_time_diff_ms( t2, t1 ) );

  // Parse class-to-idx mapping file (reusing the rsdb loader)
  t1 = msh_time_now();
  int32_t rsdb_err = rsdb_load( in_rsdb, opts.in_rsdb_filename, 0 );
  msh_panic_ceprintf( rsdb_err, "Failed to open file %s\n", opts.in_rsdb_filename );
  t2 = msh_time_now();
  msh_cprintf( !rsdb_err && opts.print_verbose, "IO: Read an rsdb file %s in %fms.\n",
                opts.in_rsdb_filename, msh_time_diff_ms( t2, t1 ) );
  
  // Convert pointcloud to .rsdb file
  t1 = msh_time_now();
  pointcloud_to_rsdb( in_rsdb, pointcloud, out_rsdb, &opts );
  t2 = msh_time_now();
  msh_cprintf( opts.print_verbose, "Extracting models took %f ms.\n", msh_time_diff_ms( t2, t1 ) );

  // Save out .rsdb file
  t1 = msh_time_now();
  rsdb_save( out_rsdb, opts.out_rsdb_filename, 1 );
  t2 = msh_time_now();
  msh_cprintf( opts.print_verbose, "Saving database %s took %f ms.\n", opts.out_rsdb_filename,
               msh_time_diff_ms( t2, t1) );

  // Clean-up
  rsdb_free( in_rsdb );
  rsdb_free( out_rsdb );
  rs_pointcloud_free( pointcloud, 1 );

  return 0;
}
