/*===========================================================================*
  *
  *  user_sf_IO.c
  *    
  *  Project:    PendulumSpringC
  * 
  *  Generation date: 14-Nov-2014 18:28:15
  * 
  *  (c) Universite catholique de Louvain
  *      2, Place du Levant 
  *      1348 Louvain-la-Neuve 
  *  http://www.robotran.be// 
  *  
  *===========================================================================*/

#include <stdlib.h>
#include "user_IO.h"
#include "mbs_data.h"
#include "set_output.h"

int user_synch_output(MbsData *mbs_data)
{
    return set_mbs_to_output(mbs_data);
}

UserIO* mbs_new_user_IO(UserIoInfo* ioInfo)
{
    UserIO *uvs;
    int i=0;
    //
    uvs = (UserIO*) malloc(sizeof(UserIO));
    
    return uvs;
}

void mbs_delete_user_IO(UserIO *uvs)
{

    free(uvs);
}

void mbs_get_user_IO_size(int *n_in, int *n_out, int *n_user_IO) 
{
    *n_in  = 0; 
    *n_out = 0; 
    *n_user_IO = 0; 
}

void mbs_print_user_IO(UserIO* uio)
{

}

void mbs_save_user_IO(FILE* stream, UserIO *uio)
{

}
