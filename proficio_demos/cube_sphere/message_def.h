#define MID_PRODUCER 10
#define MID_CONSUMER 11

#define MT_REQUEST_TEST_DATA 101
#define MT_TEST_DATA 102
#define MT_FORCE_FEEDBACK 103
#define MT_POSITION_FEEDBACK 104

typedef struct 
{ 
    int a; 
    int b; 
    double x; 
} MDF_TEST_DATA;

typedef struct
{
	double x;
	double y;
	double z;
} MDF_FORCE_FEEDBACK;

typedef struct
{
	double x;
	double y;
	double z;
} MDF_POSITION_FEEDBACK;
