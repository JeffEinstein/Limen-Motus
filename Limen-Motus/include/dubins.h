
#ifndef DUBINS_H
#define DUBINS_H

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

#define EDUBOK        (0)
#define EDUBCOCONFIGS (1)
#define EDUBPARAM     (2)
#define EDUBBADRHO    (3)
#define EDUBNOPATH    (4)

namespace HybridAStar {

typedef int (*DubinsWord)(double, double, double, double* );

extern DubinsWord dubins_words[];

typedef struct
{
    double qi[3];
    double param[3];
    double rho;
    int type;
} DubinsPath;

typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void* user_data);

int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path);

double dubins_path_length( DubinsPath* path );

int dubins_path_type( DubinsPath * path );

int dubins_path_sample( DubinsPath* path, double t, double q[3]);

int dubins_path_sample_many( DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize, void* user_data );

int dubins_path_endpoint( DubinsPath* path, double q[3] );

int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath );

int dubins_LSL( double alpha, double beta, double d, double* outputs );
int dubins_RSR( double alpha, double beta, double d, double* outputs );
int dubins_LSR( double alpha, double beta, double d, double* outputs );
int dubins_RSL( double alpha, double beta, double d, double* outputs );
int dubins_LRL( double alpha, double beta, double d, double* outputs );
int dubins_RLR( double alpha, double beta, double d, double* outputs );

}
#endif
