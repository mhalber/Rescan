/*
  ==============================================================================
  
  RS_DATABASE.H - WIP! 
  
  This file database the database format for rescan project files.

  #define RS_DATABASE_IMPLEMENTATION
  #include "rs_database.h"

  ==============================================================================
  DEPENDENCIES
  msh.h, msh_vec3.h, msh_ply.h

  ==============================================================================
  AUTHORS

    Maciej S. Halber (macikuh@gmail.com)
  ==============================================================================
  LICENSE

  Copyright (c) 2017-2018 Maciej S. Halber

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

*/

#ifndef RS_DATABASE_H
#define RS_DATABASE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef RSDB_STATIC
#define RSDBDEF static
#else
#define RSDBDEF extern
#endif

#if defined(MSH_SYSTEM_WINDOWS) || defined(MSH_SYSTEM_MSYS) 
#include <direct.h>
#endif

#ifdef MSH_SYSTEM_OSX
#include <sys/stat.h>
#include <sys/types.h>
#endif

#define RSDB_MAX_CMD_LENGTH 128
#define RSDB_MAX_STR_LENGTH 1024

typedef struct rescan_object
{
  int32_t uidx;
  char* filename;
  int32_t class_idx;
  int32_t is_shape_prior;
  rs_pointcloud_t* shape;
} rs_object_t;

typedef struct rs_object_placement
{
  int32_t uidx;
  int32_t arrangement_idx;
  int32_t object_idx;
  int32_t pose_idx;
  msh_mat4_t pose;
  float score;
} rs_obj_plcmnt_t;

typedef struct rs_scene
{
  int32_t uidx;
  int32_t arrangement_idx;
  char* scn_filename;
  char* pose_proposal_filename;
  rs_pointcloud_t* shape;
  int32_t first_dynamic_object_uidx;
} rs_scene_t;

typedef struct rescan_database
{
  int32_t version_major;
  int32_t version_minor;
  char* model_folder;

  hashtable_t class_name_to_idx;
  hashtable_t idx_to_class_name;

  msh_array(rs_scene_t)  scenes;
  msh_array(rs_object_t) objects;
  
  msh_array(msh_array(rs_obj_plcmnt_t)) arrangements;
  msh_array(msh_array(msh_array(msh_mat4_t))) proposed_poses;
  msh_array(msh_array(msh_array(float))) proposed_poses_scores;

  int32_t pointcloud_data_present;

  // int32_t n_classes;
} rsdb_t;


rsdb_t*      rsdb_init();
int32_t      rsdb_add_class(rsdb_t* rsdb, const char* class_name, const int class_idx);
int32_t      rsdb_add_object(rsdb_t* rsdb, rs_object_t* object);
int32_t      rsdb_add_scene(rsdb_t* rsdb, rs_scene_t* scene );
int32_t      rsdb_add_arrangement(rsdb_t* rsdb,
                                  msh_array(rs_obj_plcmnt_t) arrangement );

rs_scene_t* rsdb_find_scene( rsdb_t* rsdb, char* scene_name );

void         rsdb_free(rsdb_t* rsdb);
int32_t      rsdb_load(rsdb_t* rsdb, const char* db_filename, int32_t load_pointclouds );
int32_t      rsdb_save(rsdb_t* rsdb, const char* db_filename, int32_t save_objects);

rs_object_t  rsdb_object_init();
void         rsdb_object_free(rs_object_t* object, int free_shape);
rs_object_t* rsdb_object_find(rsdb_t* rsdb, int32_t uidx);

rs_scene_t   rsdb_scene_init( char* filename, int idx, int load_cloud );
void         rsdb_scene_free( rs_scene_t* scene, int free_shape );
int          rsdb_get_class_idx( const rsdb_t* rsdb, const char* class_name );
char*        rsdb_get_class_name( const rsdb_t* rsdb, const int class_idx );
int          rsdb_is_class_static( const rsdb_t* rsdb, const int class_idx );
int          rsdb_is_object_static( const rsdb_t* rsdb, const int obj_idx );
int32_t      rsdb_find_uidx_of_first_dynamic_object( const rsdb_t* rsdb, const int32_t scene_idx );
void         rsdb_refine_alignment_of_objects_to_scene( const rsdb_t* rsdb, const int32_t scene_idx, const int32_t skip_static );
#ifdef __cplusplus
}
#endif

#endif /*RS_DATABASE_H*/




////////////////////////////////////////////////////////////////////////////////////////////////////



#ifdef RS_DATABASE_IMPLEMENTATION

typedef enum rsdb_error_codes
{
  RSDB_NO_ERRORS = 0,
  RSDB_FILE_LOAD_ERR, /*1*/
  RSDB_FILE_SAVE_ERR, /*2*/
  RSDB_VERSION_CMD_ERR, /*3*/
  RSDB_CLASS_CMD_ERR, /*4*/
  RSDB_SCENE_CMD_ERR, /*5*/
  RSDB_MODEL_FOLDER_CMD_ERR, /*6*/
  RSDB_SEQ_NAME_CMD_ERR, /*7*/
  RSDB_OBJECT_CMD_ERR, /*8*/
  RSDB_SHAPE_PRIOR_CMD_ERR, /*9*/
  RSDB_N_ARRANGEMENTS_CMD_ERR, /*10*/
  RSDB_POSE_CMD_ERR, /*11*/
  RSDB_INVALID_PTR, /*12*/
  RSDB_UNRECOGNIZED_CMD_ERR,/*13*/
  RSDB_POINTCLOUD_LOAD_ERR,/*14*/
  RSDB_NOT_IMPLEMENTED
} rsdb_err_t;


// djb2 by dan bernstein
uint64_t
msh_hash_string(const char *str)
{
  uint64_t hash = 5381;
  int c;

  while ((c = *str++))
    hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

  return hash;
}

int32_t
rsdb_find_uidx_of_first_dynamic_object( const rsdb_t* rsdb, const int32_t scene_idx )
{
  int32_t min_uidx = 1024;

  if( scene_idx < msh_array_len( rsdb->arrangements ) )
  {
    msh_array(rs_obj_plcmnt_t) arrangement = rsdb->arrangements[scene_idx];
    for( size_t j = 0; j < msh_array_len(arrangement); ++j )
    {
      if( !rsdb_is_object_static( rsdb, arrangement[j].object_idx) &&
          arrangement[j].uidx < min_uidx ) 
      {
        min_uidx = arrangement[j].uidx;
      }
    }
  }
  rsdb->scenes[scene_idx].first_dynamic_object_uidx = min_uidx;
  
  return min_uidx;
}

void
rsdb_refine_alignment_of_objects_to_scene( const rsdb_t* rsdb, const int32_t scene_idx, const int32_t skip_static )
{
#ifdef ICP_H
  for( size_t i = 0; i < msh_array_len( rsdb->arrangements[scene_idx] ); ++i )
  {
    int32_t lvl = 2;
    rs_obj_plcmnt_t* plcmnt = &rsdb->arrangements[scene_idx][i];
    if( skip_static && rsdb_is_object_static( rsdb, plcmnt->object_idx ) ) { continue; }
    rs_pointcloud_t* obj = rsdb->objects[plcmnt->object_idx].shape;
    rs_pointcloud_t* scn = rsdb->scenes[scene_idx].shape;
    icp_align( obj->positions[lvl], obj->normals[lvl], obj->n_pts[lvl],
               scn->positions[lvl], scn->normals[lvl], scn->n_pts[lvl],
               &plcmnt->pose, msh_mat4_identity(), 0.075f, msh_deg2rad(50.0f), 0);
  }
#endif
}

int
rsdb_get_class_idx( const rsdb_t* rsdb, const char* class_name )
{
  uint64_t* class_idx_ptr = (uint64_t*)hashtable_find( &rsdb->class_name_to_idx, 
                                                        msh_hash_string(class_name) );
  int32_t class_idx = -1;
  if( class_idx_ptr ) class_idx = *(int32_t*)class_idx_ptr;
  return class_idx;
}

char*
rsdb_get_class_name( const rsdb_t* rsdb, const int class_idx )
{
  return (char*)hashtable_find( &rsdb->idx_to_class_name, (uint64_t)class_idx );
}

int
rsdb_is_object_static( const rsdb_t* rsdb, const int obj_idx )
{
  int32_t class_idx = rsdb->objects[obj_idx].class_idx;
  return rsdb_is_class_static( rsdb, class_idx );
}

int
rsdb_is_class_static( const rsdb_t* rsdb, const int class_idx )
{
  static int32_t wall_idx           = rsdb_get_class_idx( rsdb, "wall" );
  static int32_t floor_idx          = rsdb_get_class_idx( rsdb, "floor" );
  static int32_t ceiling_idx        = rsdb_get_class_idx( rsdb, "ceiling" );
  static int32_t door_idx           = rsdb_get_class_idx( rsdb, "door" );
  static int32_t window_idx         = rsdb_get_class_idx( rsdb, "window" );
  static int32_t picture_idx        = rsdb_get_class_idx( rsdb, "picture" );
  static int32_t counter_idx        = rsdb_get_class_idx( rsdb, "counter" );
  static int32_t cabinet_idx        = rsdb_get_class_idx( rsdb, "cabinet" );
  static int32_t bookshelf_idx      = rsdb_get_class_idx( rsdb, "bookshelf");
  static int32_t shelves_idx        = rsdb_get_class_idx( rsdb, "shelves");
  static int32_t unlabelled_idx     = rsdb_get_class_idx( rsdb, "unlabelled");
  static int32_t other_idx          = rsdb_get_class_idx( rsdb, "other");

  int is_static = 0;
  is_static = is_static || ( ( wall_idx >= 0 ) ? ( class_idx == wall_idx ) : 0 );
  is_static = is_static || ( ( floor_idx >= 0 ) ? ( class_idx == floor_idx ) : 0 );
  is_static = is_static || ( ( ceiling_idx >= 0 ) ? ( class_idx == ceiling_idx ) : 0 );
  is_static = is_static || ( ( door_idx >= 0 ) ? ( class_idx == door_idx ) : 0 );
  is_static = is_static || ( ( window_idx >= 0 ) ? ( class_idx == window_idx ) : 0 );
  is_static = is_static || ( ( picture_idx >= 0 ) ? ( class_idx == picture_idx ) : 0 );
  is_static = is_static || ( ( counter_idx >= 0 ) ? ( class_idx == counter_idx ) : 0 );
  is_static = is_static || ( ( cabinet_idx >= 0 ) ? ( class_idx == cabinet_idx ) : 0 );
  is_static = is_static || ( ( bookshelf_idx >= 0 ) ? ( class_idx == bookshelf_idx ) : 0 );
  is_static = is_static || ( ( shelves_idx >= 0 ) ? ( class_idx == shelves_idx ) : 0 );
  is_static = is_static || ( ( unlabelled_idx >= 0 ) ? ( class_idx == unlabelled_idx ) : 0 );
  is_static = is_static || ( ( other_idx >= 0 ) ? ( class_idx == other_idx ) : 0 );

  return is_static;
}

int32_t
rsdb__parse_rsdb_cmd(char* line, rsdb_t* rsdb)
{
  char cmd[RSDB_MAX_CMD_LENGTH];
  if( sscanf(line, "%s %d.%d", cmd, &rsdb->version_major, &rsdb->version_minor) != 3 )
  {
    return RSDB_VERSION_CMD_ERR;
  }
  return RSDB_NO_ERRORS;
}

int32_t
rsdb__parse_class_cmd(char* line, rsdb_t* rsdb)
{
  char cmd[RSDB_MAX_CMD_LENGTH] = {0};
  char class_name[64] = {0};
  int class_idx = -1;
  int count = sscanf(line, "%s %s %d", cmd, class_name, &class_idx);
  if( count != 3)
  {
    return RSDB_CLASS_CMD_ERR;
  }
  hashtable_insert(&rsdb->class_name_to_idx, msh_hash_string(class_name), &class_idx);
  hashtable_insert(&rsdb->idx_to_class_name, (HASHTABLE_U64)class_idx, class_name );

  return RSDB_NO_ERRORS;
}

int rsdb__parse_object_cmd_general( char* line, rsdb_t* rsdb, bool is_shape_prior )
{
  char cmd[RSDB_MAX_CMD_LENGTH] = {0};
  char object_filename[256] = {0};
  int uidx = -1;
  int class_idx = -1;
  
  if( sscanf(line, "%s %s %d %d", cmd, object_filename, &uidx, &class_idx ) != 4 )
  {
    return RSDB_OBJECT_CMD_ERR;
  }
  
  rs_object_t object  = rsdb_object_init();
  object.filename       = strdup(object_filename);
  object.uidx           = uidx;
  object.class_idx      = class_idx;
  object.is_shape_prior = is_shape_prior;
 
  msh_array_push(rsdb->objects, object);
  return RSDB_NO_ERRORS;
}

int rsdb__parse_object_cmd(char* line, rsdb_t* rsdb)
{
  return rsdb__parse_object_cmd_general(line, rsdb, 0);
}

int rsdb__parse_shape_prior_cmd(char* line, rsdb_t* rsdb)
{
  return rsdb__parse_object_cmd_general(line, rsdb, 1);
}

int rsdb__parse_n_arrangements_cmd(char* line, rsdb_t* rsdb )
{
  char cmd[RSDB_MAX_CMD_LENGTH] = {0};
  int n_arrangements = 0;
  if( sscanf( line, "%s %d", cmd, &n_arrangements ) != 2 ) { return RSDB_N_ARRANGEMENTS_CMD_ERR; }

  if( n_arrangements > 0 )
  {
    rsdb->arrangements = {0};
    msh_array_fit( rsdb->arrangements, (size_t)n_arrangements );
    for( int i = 0 ; i < n_arrangements ; ++i ) 
    { 
      msh_array(rs_obj_plcmnt_t) placement = {0}; 
      msh_array_push(rsdb->arrangements, placement);
    }
  }
  return RSDB_NO_ERRORS;
}

int rsdb__parse_model_folder_cmd(char* line, rsdb_t* rsdb )
{
  char cmd[RSDB_MAX_CMD_LENGTH] = {0};
  char folder_name[512] = {0};
  if( sscanf( line, "%s %s", cmd, folder_name ) != 2 ) { return RSDB_MODEL_FOLDER_CMD_ERR; }
  
  rsdb->model_folder = msh_strdup(folder_name);
  return RSDB_NO_ERRORS;
}

int rsdb__parse_pose_cmd( char* line, rsdb_t* rsdb )
{
  // TODO(maciej): Should I be storing pose id
  char cmd[RSDB_MAX_CMD_LENGTH] = {0};
  int placement_idx, arrangement_idx, object_idx;
  placement_idx = arrangement_idx = object_idx = -1;
  float score = 0.0f;
  msh_mat4_t T = msh_mat4_identity();
  if(sscanf(line, "%s  %d %d %d %f  %f %f %f %f  %f %f %f %f  %f %f %f %f  %f %f %f %f", 
            cmd, &placement_idx, &arrangement_idx, &object_idx, &score,
                   &T.col[0].x, &T.col[1].x, &T.col[2].x, &T.col[3].x,
                   &T.col[0].y, &T.col[1].y, &T.col[2].y, &T.col[3].y,
                   &T.col[0].z, &T.col[1].z, &T.col[2].z, &T.col[3].z,
                   &T.col[0].w, &T.col[1].w, &T.col[2].w, &T.col[3].w  ) != 21)
  {
    return RSDB_POSE_CMD_ERR;
  }

  rs_obj_plcmnt_t placement = { placement_idx, arrangement_idx, object_idx, 0, T, score };
  msh_array_push( rsdb->arrangements[arrangement_idx], placement );
 
  return RSDB_NO_ERRORS;
}


int rsdb__parse_scene_cmd( char* line, rsdb_t* rsdb )
{
  char cmd[RSDB_MAX_CMD_LENGTH] = {0};
  rs_scene_t s = {};
  char scn_filename[1024] = {0};
  char pp_filename[1024] = {0};

  if( sscanf( line, "%s  %d %d %s %s", 
              cmd, &s.uidx, &s.arrangement_idx, scn_filename, pp_filename ) != 5 )
  {
    return RSDB_SCENE_CMD_ERR;
  }
  s.scn_filename = msh_strdup( scn_filename );
  if( !strcmp(pp_filename, "none") )  { s.pose_proposal_filename = NULL; }
  else                                { s.pose_proposal_filename = msh_strdup( pp_filename ); }

  msh_array_push( rsdb->scenes, s );
  msh_array_push( rsdb->proposed_poses, NULL );
  msh_array_push( rsdb->proposed_poses_scores, NULL );
  msh_array_push( rsdb->arrangements, NULL );

  return RSDB_NO_ERRORS;
}

int
rsdb__parse_command( char* cmd, char* line, rsdb_t* rsdb )
{

  if(!strcmp(cmd,"rsdb"))            { return rsdb__parse_rsdb_cmd( line, rsdb ); }
  if(!strcmp(cmd,"class"))           { return rsdb__parse_class_cmd( line, rsdb ); }
  if(!strcmp(cmd,"object"))          { return rsdb__parse_object_cmd( line, rsdb ); }
  if(!strcmp(cmd,"shape_prior"))     { return rsdb__parse_shape_prior_cmd( line, rsdb ); }
  if(!strcmp(cmd,"n_arrangements"))  { return rsdb__parse_n_arrangements_cmd( line, rsdb ); }
  if(!strcmp(cmd,"model_folder"))    { return rsdb__parse_model_folder_cmd( line, rsdb ); }
  if(!strcmp(cmd,"pose"))            { return rsdb__parse_pose_cmd( line, rsdb ); }
  if(!strcmp(cmd,"scene"))           { return rsdb__parse_scene_cmd( line, rsdb ); }
  return RSDB_UNRECOGNIZED_CMD_ERR;
}

int rsdb_load(rsdb_t* rsdb, const char* filename, int load_pointclouds)
{
  int error_code = 0;
  rsdb->pointcloud_data_present = load_pointclouds;

  // Read in database file
  FILE* fp = fopen(filename, "r");
  if(!fp) { return RSDB_FILE_LOAD_ERR; }
  char line[RSDB_MAX_STR_LENGTH];
  int line_no = 0;

  while( fgets( &line[0], RSDB_MAX_STR_LENGTH, fp) )
  {
    line_no++;
    char cmd[RSDB_MAX_CMD_LENGTH];
    if(sscanf(line, "%s", cmd) != (unsigned) 1 ) {continue;}

    error_code = rsdb__parse_command( cmd, line, rsdb );
    if( error_code != RSDB_NO_ERRORS )
    {
      if( error_code == RSDB_UNRECOGNIZED_CMD_ERR )
      {
        printf("Unrecognized command at line %d : %s\n", line_no, line);
      }
      else { printf("Error: %d\n", error_code); break; }
    }
  }
  fclose(fp);

  // Compute some data information for ease of future access
  int n_objects = msh_array_len( rsdb->objects );
  int n_scenes  = msh_array_len( rsdb->scenes );
  int n_arrangements  = msh_array_len( rsdb->arrangements );
  int n_classes = hashtable_count( &rsdb->class_name_to_idx );

  assert( n_scenes == n_arrangements );
  // Load pointclouds if requested
  if( rsdb->pointcloud_data_present )
  {
    // First gather filenames
    msh_array(const char*) pointcloud_filenames = NULL;;
    for( int i = 0; i < n_objects; ++i )
    {
      char path[1024];
      snprintf( path, 1024, "%s%c%s",
                rsdb->model_folder, MSH_FILE_SEPARATOR, rsdb->objects[i].filename);
      msh_array_push( pointcloud_filenames, msh_strdup(path) );
    }
    for( int i = 0; i <  n_scenes ; i++ )
    {
      rs_scene_t* s = &rsdb->scenes[i];
      msh_array_push( pointcloud_filenames, msh_strdup( s->scn_filename ));
      rsdb_find_uidx_of_first_dynamic_object( rsdb, i );
    }
    int pointcloud_count = n_scenes + n_objects;
    rs_pointcloud_t* pointclouds = rs_pointcloud_init( pointcloud_count );
    int load_error = rs_pointcloud_from_files( pointclouds, 
                                               pointcloud_filenames, pointcloud_count, 1 );
    if( load_error ) { error_code = RSDB_POINTCLOUD_LOAD_ERR; }
    if( error_code == RSDB_NO_ERRORS )
    {
      for( int i = 0; i < n_objects; ++i ) 
      { 
        rs_object_t* object = &rsdb->objects[i];
        object->shape       = &pointclouds[i]; 
      }
      for( int i = n_objects; i < n_objects + n_scenes; ++i ) 
      { 
        rs_scene_t* scene = &rsdb->scenes[i - n_objects];
        scene->shape       = &pointclouds[i]; 
      }
    }
    for( int i = 0; i < pointcloud_count; ++i ) 
    { 
      free( (void*)pointcloud_filenames[i] );
    }
    msh_array_free( pointcloud_filenames );
  }

  return error_code;
}

void
rsdb__save_object(const char* filename, rs_object_t* object )
{
  int32_t lvl = 0;
  rs_pointcloud_t *shape = object->shape;
  rs_pointcloud__save_ply(filename, shape, lvl);
}

int32_t
rsdb_save(rsdb_t* rsdb, const char* db_filename, int save_objects)
{
  free( rsdb->model_folder );
  rsdb->model_folder = create_model_folder_name( db_filename );

  // create folder
  if( save_objects )
  {
#if MSH_PLATFORM_WINDOWS
    _mkdir(rsdb->model_folder);
#else
    mkdir(rsdb->model_folder, 0777);
#endif
  }
  int32_t err = RSDB_NO_ERRORS;
  FILE* fp = fopen(db_filename, "w");
  if(!fp) { return RSDB_FILE_SAVE_ERR; }
  fprintf(fp, "rsdb %d.%d\n", rsdb->version_major, rsdb->version_minor);
  fprintf(fp, "model_folder %s\n", rsdb->model_folder);

  int32_t itc_count = hashtable_count( &rsdb->idx_to_class_name );
  HASHTABLE_U64 const* itc_keys = hashtable_keys( &rsdb->idx_to_class_name );
  for( int32_t i = 0; i < itc_count; ++i )
  {
    char* name = (char*)hashtable_find( &rsdb->idx_to_class_name, itc_keys[i]);
    fprintf(fp, "class %s %zu\n", name, itc_keys[i] );
  }

   for( size_t i = 0; i < msh_array_len( rsdb->scenes ); ++i)
  {
    rs_scene_t* scene = &rsdb->scenes[i];
    char prp_filename[1024] = {0};
    char pp_filename[1024] = {0};
    if( scene->pose_proposal_filename == NULL ) { sprintf(pp_filename, "none"); }
    else                                        { sprintf(pp_filename, "%s", scene->pose_proposal_filename); }
    fprintf(fp, "scene %d %d %s %s %s\n", 
            scene->uidx, scene->arrangement_idx, 
            scene->scn_filename, pp_filename, prp_filename );
  }

  for( size_t i = 0; i < msh_array_len(rsdb->objects); ++i)
  {
    rs_object_t* object = &rsdb->objects[i];
    if( object->is_shape_prior )
    {
      fprintf( fp, "shape_prior %s %d %d\n", object->filename, object->uidx, object->class_idx );
    }
    else
    {
      fprintf( fp, "object %s %d %d\n", object->filename, object->uidx, object->class_idx );
    }
    if( save_objects ) 
    {
      char object_name[1024];
      snprintf(object_name, 1024, "%s%c%s", rsdb->model_folder, MSH_FILE_SEPARATOR, object->filename);
      printf("Writing: %s\n", object_name );
      rsdb__save_object( object_name, object );
    }
  }
  int n_arrangements = msh_array_len(rsdb->arrangements);
  fprintf(fp, "n_arrangements %d\n", n_arrangements );
  for( int i = 0 ; i < n_arrangements; ++i )
  {
    msh_array(rs_obj_plcmnt_t) arrangement = rsdb->arrangements[i];
    for( size_t j = 0 ; j < msh_array_len(arrangement); ++j )
    {
      rs_obj_plcmnt_t* placement = &arrangement[j];
      msh_mat4_t T = placement->pose;
      fprintf(fp, "pose %d %d %d %f   %f %f %f %f  %f %f %f %f  %f %f %f %f  %f %f %f %f\n",
            placement->uidx, i, placement->object_idx, placement->score,
            T.col[0].x, T.col[1].x, T.col[2].x, T.col[3].x,
            T.col[0].y, T.col[1].y, T.col[2].y, T.col[3].y,
            T.col[0].z, T.col[1].z, T.col[2].z, T.col[3].z,
            T.col[0].w, T.col[1].w, T.col[2].w, T.col[3].w );
    }
  }
  fclose(fp);
  return err;
}

rsdb_t* 
rsdb_init()
{
  rsdb_t* rsdb                    = (rsdb_t*)malloc(sizeof(rsdb_t));
  rsdb->version_major             = 1;
  rsdb->version_minor             = 0;
  rsdb->objects                   = NULL;
  rsdb->arrangements              = NULL;
  rsdb->scenes                    = NULL;
  rsdb->proposed_poses            = NULL;
  rsdb->proposed_poses_scores     = NULL;
  rsdb->model_folder              = NULL;
  hashtable_init(&rsdb->class_name_to_idx, sizeof(int), 30, NULL);
  hashtable_init(&rsdb->idx_to_class_name, 512*sizeof(char), 30, NULL);
  return rsdb;
}

int 
rsdb_add_class(rsdb_t* rsdb, const char* class_name, const int class_idx)
{
  // check if does not exists already
  int* class_ind = (int*)hashtable_items( &rsdb->class_name_to_idx );
  for(int i = 0; i < hashtable_count( &rsdb->class_name_to_idx ); ++i)
  {
    char* existing_class_name = (char*)hashtable_find( &rsdb->idx_to_class_name, class_ind[i] );
    if( !strcmp( existing_class_name, class_name ) ) { return i; }
  }

  hashtable_insert(&rsdb->class_name_to_idx, msh_hash_string(class_name), &class_idx);
  hashtable_insert(&rsdb->idx_to_class_name, (HASHTABLE_U64)class_idx, class_name );

  return hashtable_count( &rsdb->class_name_to_idx )-1;
}

int32_t
rsdb_add_object(rsdb_t* rsdb, rs_object_t* object )
{
  // check if does not exists already
  for( size_t i = 0; i < msh_array_len(rsdb->objects); ++i )
  {
    if( rsdb->objects[i].uidx == object->uidx )
      return rsdb->objects[i].uidx;
  }
  msh_array_push(rsdb->objects, *object);
  return msh_array_len(rsdb->objects)-1;
}

int32_t
rsdb_add_scene( rsdb_t* rsdb, rs_scene_t* scene )
{
  // check if does not exists already
  for( size_t i = 0; i < msh_array_len(rsdb->scenes); ++i)
  {
    if( rsdb->scenes[i].uidx == scene->uidx )
      return rsdb->scenes[i].uidx;
  }
  msh_array_push( rsdb->scenes, *scene);
  return msh_array_len(rsdb->scenes)-1;
}


rs_scene_t* 
rsdb_find_scene( rsdb_t* rsdb, char* name )
{
  rs_scene_t* ret = NULL;
  for( size_t i = 0; i < msh_array_len(rsdb->scenes); ++i)
  {
    if( !strcmp( rsdb->scenes[i].scn_filename, name ) )
    {
      ret = &rsdb->scenes[i];
      break;
    }
  }
  return ret;
}

int32_t
rsdb_add_arrangement(rsdb_t* rsdb, msh_array(rs_obj_plcmnt_t) arrangement )
{
  msh_array_push(rsdb->arrangements, arrangement);
  return RSDB_NO_ERRORS;
}

void 
rsdb_free(rsdb_t* rsdb)
{
  for( size_t i = 0; i < msh_array_len(rsdb->scenes); ++i)
  {
    rs_scene_t *scene = &rsdb->scenes[i];
    rsdb_scene_free(scene, rsdb->pointcloud_data_present);
  }
  for( size_t i = 0; i < msh_array_len(rsdb->arrangements); ++i)
  {
    if( rsdb->arrangements[i]) msh_array_free( rsdb->arrangements[i] );
  }

  if( rsdb->objects )      msh_array_free(rsdb->objects);
  if( rsdb->scenes )       msh_array_free(rsdb->scenes);
  if( rsdb->arrangements ) msh_array_free(rsdb->arrangements);
  hashtable_term(&rsdb->class_name_to_idx);
  hashtable_term(&rsdb->idx_to_class_name);

  free(rsdb);
  rsdb = NULL;
}

rs_object_t
rsdb_object_init()
{
  rs_object_t object;
  object.filename       = NULL;
  object.shape          = NULL;
  object.uidx           = -1;
  object.class_idx      = -1;
  object.is_shape_prior = -1;
  return object;
}

void
rsdb_object_free(rs_object_t* object, int free_shape)
{
  if(!object) return;
  if(object->filename)             free(object->filename);
  if(object->shape && free_shape)  rs_pointcloud_free( object->shape, 1 );
}

rs_scene_t
rsdb_scene_init( char* filename, int idx, int load_scene )
{
  rs_scene_t scene = {};
  scene.scn_filename           = filename;
  scene.pose_proposal_filename = NULL;
  scene.shape                  = NULL;
  scene.uidx                   = idx;
  scene.arrangement_idx        = idx;
  if( load_scene )
  {
    rs_pointcloud_t* pc = rs_pointcloud_init(1);
    int pc_load_err = rs_pointcloud_from_file( pc, filename );
    if( pc_load_err )
    {
      printf( "IO: Could not open scene file %s\n", filename );
    }
    else
    {
      scene.shape = pc;
    }
  }
  return scene;
}

void
rsdb_scene_free(rs_scene_t* scene, int free_shape)
{
  if(!scene) return;
  if(scene->scn_filename)           free(scene->scn_filename);
  if(scene->pose_proposal_filename) free(scene->pose_proposal_filename);
  if(scene->shape && free_shape)    rs_pointcloud_free( scene->shape, 1 );
}

rs_object_t* 
rsdb_object_find(rsdb_t* rsdb, int uidx)
{
  for( size_t i = 0; i < msh_array_len(rsdb->objects); ++i)
  {
    if( rsdb->objects[i].uidx == uidx ) { return &rsdb->objects[i]; }
  }
  return NULL;
}

#endif /*RS_DATABASE_IMPLEMENTATION*/