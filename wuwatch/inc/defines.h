#ifndef _WU_DEFINES_H_
#define _WU_DEFINES_H_ 1

/*
 * Some useful typedefs/macros
 */
#define INC_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(1)
#define GET_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(0)
#define DEC_NUM_SHM_CLIENTS() inc_dec_num_shm_clients(-1)
/*
 * Convert 'u64' to 'unsigned long long'
 * Required to get around pesky "invalid format" gcc compiler
 * warnings.
 */
#define TO_ULL(x) (unsigned long long)(x)
/*
 * Convert an arg to 'unsigned long'
 */
#define TO_UL(x) (unsigned long)(x)

#endif // _WU_DEFINES_H_
