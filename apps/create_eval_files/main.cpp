#define MSH_STD_INCLUDE_LIBC_HEADERS
#define MSH_STD_IMPLEMENTATION
#define MSH_PLY_IMPLEMENTATION
#define MSH_ARGPARSE_IMPLEMENTATION
#define MSH_VEC_MATH_IMPLEMENTATION
#define MSH_HASH_GRID_IMPLEMENTATION
#define MSH_GEOMETRY_IMPLEMENTATION
#define RS_POINTCLOUD_IMPLEMENTATION
#define HASHTABLE_IMPLEMENTATION
#define FILEPATH_HELPERS_IMPLEMENTATION


#include "msh/msh_std.h"
#include "msh/msh_argparse.h"
#include "msh/msh_vec_math.h"
#include "msh/msh_geometry.h"
#include "msh/msh_ply.h"
#include "msh/msh_hash_grid.h"
#include "mg/hashtable.h"
#include "rs_pointcloud.h"
#include "filepath_helpers.h"

typedef struct options
{
  char* input_ply_filename;
  char* evaluation_files_folder;
  bool print_verbose;
} options_t;

#define MAX_STR_LENGTH 1024

const char*
create_semantic_segmentation_filename( const char* scannet_folder, const char* input_ply_filename )
{
  const char* base_name         = msh_path_basename( input_ply_filename );
  const char* ext               = msh_path_get_ext( input_ply_filename );
  char seq_name[MAX_STR_LENGTH] = {0};
  msh_strcpy_range( seq_name, base_name, 0, ext - base_name - 1 );

  const char* folder_name    = extract_method_name( input_ply_filename );

  char output_filename[ MAX_STR_LENGTH ] = {0};
  msh_path_join( output_filename, MAX_STR_LENGTH, 3, scannet_folder, "semantic_label", folder_name );
  msh_create_directory( output_filename );
  msh_path_join( output_filename, MAX_STR_LENGTH, 2, output_filename, seq_name );

  msh_strcpy_range( output_filename, ".txt", strlen(output_filename), MAX_STR_LENGTH );

  free( (void*)folder_name );

  return msh_strdup(output_filename);
}

const char*
create_base_instance_segmentation_filename( const char* scannet_folder, const char* input_ply_filename,
                                            const char* evaluation_type_folder )
{
  const char* base_name         = msh_path_basename( input_ply_filename );
  const char* ext               = msh_path_get_ext( input_ply_filename );
  char seq_name[MAX_STR_LENGTH] = {0};
  msh_strcpy_range( seq_name, base_name, 0, ext - base_name - 1 );

  const char* folder_name    = extract_method_name( input_ply_filename );

  char output_filename[ MAX_STR_LENGTH ];
  msh_path_join( output_filename, MAX_STR_LENGTH, 3, scannet_folder, evaluation_type_folder, folder_name );
  msh_create_directory( output_filename );
  msh_path_join( output_filename, MAX_STR_LENGTH, 2, output_filename, seq_name );

  msh_strcpy_range( output_filename, ".txt", strlen(output_filename), MAX_STR_LENGTH );
  free( (void*)folder_name );
  return msh_strdup(output_filename);
}

const char*
create_sub_instance_segmentation_filename( const char* scannet_folder, const char* input_ply_filename, int32_t instance_id )
{
  const char* base_name         = msh_path_basename( input_ply_filename );
  const char* ext               = msh_path_get_ext( input_ply_filename );
  char seq_name[MAX_STR_LENGTH] = {0};
  msh_strcpy_range( seq_name, base_name, 0, ext - base_name - 1 );

  const char* folder_name    = extract_method_name( input_ply_filename );
  char instance_str[MAX_STR_LENGTH];
  snprintf(instance_str, MAX_STR_LENGTH, "%03d", instance_id );
  int32_t n = 0;
  char output_filename[ MAX_STR_LENGTH ] = {0};
  msh_path_join( output_filename, MAX_STR_LENGTH, 4, scannet_folder, "semantic_instance", folder_name, "predicted_masks" );
  msh_create_directory( output_filename );
  msh_path_join( output_filename, MAX_STR_LENGTH, 2, output_filename, seq_name );
  n = msh_strcpy_range( output_filename, "_", strlen(output_filename), MAX_STR_LENGTH );
  n = msh_strcpy_range( output_filename, instance_str, n, MAX_STR_LENGTH );
  n = msh_strcpy_range( output_filename, ".txt", n, MAX_STR_LENGTH );

  free( (void*)folder_name );

  return msh_strdup(output_filename);
}

int32_t
write_semantic_segmentation_file( const rs_pointcloud_t* pc, const char* filename )
{
  int32_t lvl = 0;
  FILE* fp = fopen( filename, "w" );
  if( fp )
  {
    for( size_t i = 0; i < pc->n_pts[lvl]; ++i )
    {
      fprintf( fp, "%d\n", pc->class_ids[lvl][i] );
    }
    fclose(fp);
    return 0;
  }
  return 1;
}

int32_t
write_mask_file( const rs_pointcloud_t* pc, const char* filename, msh_array(int32_t) inliers )
{
  int32_t lvl = 0;
  FILE* fp = fopen(filename, "w");
  if( fp )
  {
    size_t buf_size = pc->n_pts[lvl] * sizeof(int32_t);
    int32_t* buf = (int32_t*)malloc( buf_size );
    memset(buf, 0, buf_size);
    for( size_t i = 0; i < msh_array_len(inliers); ++i )
    {
      buf[ inliers[i] ] = 1;
    }
    for( size_t i = 0; i < pc->n_pts[lvl]; ++i )
    {
      fprintf(fp, "%d\n", buf[i] );
    }
    free(buf);
    fclose(fp);
    return 0;
  }
  return 1;
}

int32_t
write_instance_transfer_file( const char* filename, const rs_pointcloud_t* pc, int lvl )
{
  FILE* fp = fopen( filename, "w" );
  if( fp )
  {
    for( size_t i = 0; i < pc->n_pts[lvl]; ++i )
    {
      int32_t instance_id = pc->instance_ids[lvl][i];
      int32_t class_id    = pc->class_ids[lvl][i];
      int32_t new_instance_id = instance_id == 1024 ? 0 : instance_id + 1;
      int32_t out_instance_id = 1000 * class_id + new_instance_id;
      fprintf( fp, "%d\n", out_instance_id );
    }    
    fclose(fp);
    return 0;
  }
  return 1;
}



int32_t
write_instance_segmentation_files( const rs_pointcloud_t* pc,
                                   const char* scannet_folder,
                                   const char* ply_filename )
{

  int32_t lvl = 0;
  int32_t is_gt = 0;
  const char* semantic_instance_filename = create_base_instance_segmentation_filename( scannet_folder,
                                                                     ply_filename, "semantic_instance" );
  const char* instance_transfer_filename = create_base_instance_segmentation_filename( scannet_folder,
                                                                     ply_filename, "instance_transfer" );
  const char* folder_name = extract_method_name( ply_filename );

  if( !strcmp( folder_name, "gt_segmentation" ) )
  {
    is_gt = 1;
  }
  free( (void*)folder_name );

  if( is_gt )
  {
    // For semantic instance, scannet uses differetn types of files for ground truth and predictions.
    int err = 0;
    err = write_instance_transfer_file( semantic_instance_filename, pc, lvl );
    if( err ) { return 1; }
    err = write_instance_transfer_file( instance_transfer_filename, pc, lvl );
    if( err ) { return 1; }
    return 0;
  }
  else
  {
    write_instance_transfer_file( instance_transfer_filename, pc, lvl );

    FILE* fp = fopen( semantic_instance_filename, "w" );
    if( fp )
    {
      msh_array( msh_array(int32_t) ) instances_inliers = {0};
      msh_map_t instance_id_to_inliers = {0};

      for( size_t i = 0; i < pc->n_pts[lvl]; ++i )
      {
        uint64_t instance_id = pc->instance_ids[lvl][i];
        if( instance_id < 1024 )
        {
          uint64_t* instance_inliers_idx = msh_map_get( &instance_id_to_inliers, instance_id );
          if( instance_inliers_idx )
          {
            msh_array(int32_t) cur_inliers = instances_inliers[*instance_inliers_idx];
            msh_array_push( cur_inliers, i );
            instances_inliers[*instance_inliers_idx] = cur_inliers;
          }
          else
          {
            msh_array(int32_t) cur_inliers = {0};
            msh_array_push( cur_inliers, (int32_t)i );
            uint64_t cur_inlier_idx = msh_array_len( instances_inliers );
            msh_array_push( instances_inliers, cur_inliers );
            msh_map_insert( &instance_id_to_inliers, instance_id, cur_inlier_idx );
          }
        }
      }

      uint64_t* unique_instances = NULL;
      msh_map_get_iterable_keys( &instance_id_to_inliers, &unique_instances );
      for( size_t i = 0; i < msh_map_len( &instance_id_to_inliers ); ++i )
      {
        uint64_t instance_id  = unique_instances[i];
        uint64_t* inliers_idx = msh_map_get( &instance_id_to_inliers, instance_id );
        msh_array( int32_t) cur_inliers = instances_inliers[*inliers_idx];
        assert( cur_inliers );
        int32_t class_id = pc->class_ids[lvl][cur_inliers[0]];
        
        const char* filename = create_sub_instance_segmentation_filename( scannet_folder,
                                                                          ply_filename,
                                                                          (int32_t)instance_id );
        msh_panic_ceprintf( write_mask_file( pc, filename, cur_inliers ),
                            "Failed to write file %s\n", filename );
        const char* rel_path = strstr( filename, "predicted_masks" );
        fprintf( fp, "%s %d %f\n", rel_path, (int32_t)class_id, 1.0f );
      }
    }
    fclose(fp);
    return 0;
  }
}

int32_t
parse_arguments(int argc, char** argv, options_t* opts )
{
  // Defaults
  opts->input_ply_filename    = NULL;
  opts->evaluation_files_folder = NULL;
  opts->print_verbose         = 1;
  
  // Command line
  msh_argparse_t parser;
  msh_ap_init( &parser, "create_eval_files", 
               "Converts rescan ply files to .txt files consumed by scannet evaluation scripts" );
  msh_ap_add_string_argument( &parser, "input_ply_filename", NULL, "Input .ply filename",
                              &opts->input_ply_filename, 1 );
  msh_ap_add_string_argument( &parser, "evaluation_files_folder", NULL, "Folder to store evaluation files in",
                              &opts->evaluation_files_folder, 1 );
  msh_ap_add_bool_argument( &parser, "--verbose", "-v", "Print verbose information",
                            &opts->print_verbose, 0 );

  return msh_ap_parse( &parser, argc, argv );
}

int main( int argc, char** argv )
{
  uint64_t t1, t2;
  options_t opts;
  if( !parse_arguments( argc, argv, &opts ) ) { return 0; }

  rs_pointcloud_t* scene = rs_pointcloud_init(1);

  t1 = msh_time_now();
  msh_panic_ceprintf( rs_pointcloud__load_ply( opts.input_ply_filename, scene, 0 ),
                      "Failed to load file %s\n", opts.input_ply_filename );
  t2 = msh_time_now();
  msh_cprintf( opts.print_verbose, "RESCAN2SCANNET: Time to read pointcloud: %fms.\n", 
                                    msh_time_diff_ms(t2, t1) );
  
  const char* sem_output_filename = create_semantic_segmentation_filename( 
                                                                        opts.evaluation_files_folder, 
                                                                        opts.input_ply_filename );
  t1 = msh_time_now();
  msh_panic_ceprintf( write_semantic_segmentation_file( scene, sem_output_filename ),
                      "Failed to save file %s\n", sem_output_filename );
  t2 = msh_time_now();
  msh_cprintf( opts.print_verbose, "RESCAN2SCANNET: Time to write semantic output: %fms.\n",
                                    msh_time_diff_ms(t2, t1) );

  t1 = msh_time_now();
  msh_panic_ceprintf( write_instance_segmentation_files( scene, opts.evaluation_files_folder, opts.input_ply_filename ),
                      "Failed to save instance segementation files\n" );
  t2 = msh_time_now();
  msh_cprintf( opts.print_verbose, "RESCAN2SCANNET: Time to write instance output: %fms.\n",
                                    msh_time_diff_ms(t2, t1) );
  

  return 0;
}