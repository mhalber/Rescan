#ifndef FILEPATH_HELPERS_H
#define FILEPATH_HELPERS_H


char* create_output_segmentation_scene_filename( char* model_folder );
char* create_sequence_name( const char* scene_name );
char* create_method_name( const char* scene_name, const char* seq_name );
char* create_pose_proposal_filename( const char* rsdb_filename );
char* create_model_folder_name( const char* rsdb_filename );

#endif/*FILEPATH_HELPERS_H*/

#ifdef  FILEPATH_HELPERS_IMPLEMENTATION

char*
create_output_segmentation_scene_filename( char* model_folder_filename )
{
  char* scene_filename = (char*)malloc( 1024 );
  char folder_name[1024] = {0};
  char base_name[1024] = {0};
  char predictions_folder[1024] = {0};

  const char* separator_a_ptr = strrchr(model_folder_filename, '/' );
  const char* separator_b_ptr = strrchr(model_folder_filename, '\\' );
  const char* separator_ptr = separator_a_ptr > separator_b_ptr ? separator_a_ptr : separator_b_ptr;
  size_t separator_pos = separator_ptr ? separator_ptr - model_folder_filename + 1 : 0;

  strncpy( folder_name, model_folder_filename, separator_pos-1 );
  strncpy( base_name, model_folder_filename+separator_pos, 1023);
  sprintf( predictions_folder, "%s%cpredictions", folder_name, MSH_FILE_SEPARATOR );
#if MSH_PLATFORM_WINDOWS
    _mkdir(predictions_folder);
#else
    mkdir(predictions_folder, 0777);
#endif
  sprintf( scene_filename, "%s%c%s.ply", predictions_folder, MSH_FILE_SEPARATOR, base_name );
  return scene_filename;
}

char* 
extract_method_name( const char* scene_name )
{
  const char* first_separator_ptr = strchr( scene_name, MSH_FILE_SEPARATOR );
  const char* second_separator_ptr = strrchr( scene_name, MSH_FILE_SEPARATOR );
  size_t first_sep_pos = first_separator_ptr - scene_name + 1;
  size_t second_sep_pos = second_separator_ptr - scene_name + 1;
  if( second_sep_pos == first_sep_pos )
  {
    first_sep_pos = 0;
  }
  size_t length = second_sep_pos - first_sep_pos - 1;
  char* sequence_name = (char*)malloc( (length+1) * sizeof(char) );
  memset( sequence_name, 0, length+1 );
  strncpy( sequence_name, scene_name + first_sep_pos, length );
  return sequence_name;
}

char*
create_method_name( const char* scene_name, const char* seq_name )
{
  const char* base_name = msh_path_basename( scene_name );
  const char* end_base_name = base_name + strlen(seq_name) + 1;
  const char* ext = msh_path_get_ext( end_base_name );
  return msh_strndup( end_base_name, ext - end_base_name - 1 );
}

char* 
create_pose_proposal_filename( const char* rsdb_filename )
{
  char* pose_proposal_filename = (char*)malloc( 1024 );
  char folder_name[1024] = {0};
  char base_name[1024] = {0};

  memset(pose_proposal_filename, 0, 1024);
  const char* dot_ptr = strrchr(rsdb_filename, '.' );
  size_t dot_pos = dot_ptr - rsdb_filename + 1;
  const char* separator_a_ptr = strrchr(rsdb_filename, '/' );
  const char* separator_b_ptr = strrchr(rsdb_filename, '\\' );
  const char* separator_ptr = separator_a_ptr > separator_b_ptr ? separator_a_ptr : separator_b_ptr;
  size_t separator_pos = separator_ptr ? separator_ptr - rsdb_filename + 1 : 0;
  size_t name_length = dot_pos - separator_pos - 1;
  strncpy( folder_name, rsdb_filename, separator_pos - 1 );
  strncpy( base_name, rsdb_filename + separator_pos, name_length );
  sprintf( pose_proposal_filename, "%s%c%s%c%s.bin", 
                                    folder_name, MSH_FILE_SEPARATOR,
                                    base_name, MSH_FILE_SEPARATOR,
                                    base_name );
  return pose_proposal_filename; 
}

char* 
create_model_folder_name( const char* rsdb_filename )
{
  char* model_folder_name = (char*)malloc( 1024 );

  memset(model_folder_name, 0, 1024);
  const char* dot_ptr = strrchr( rsdb_filename, '.' );
  size_t dot_pos = dot_ptr - rsdb_filename + 1;

  strncpy( model_folder_name, rsdb_filename, dot_pos-1 );

  return model_folder_name;
}

#endif/*FILEPATH_HELPERS_IMPLEMENTATION*/