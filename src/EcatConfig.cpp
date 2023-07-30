//
// Created by yc on 2023/3/14.
//

#include <iomanip>
#include "EcatConfig.h"

pthread_mutex_t mutex;
pthread_cond_t cond;

///////////// Var define by Yang Luo //////////////


const double CNT_PER_RAD_HIGH = RES_HIGH * RATIO / (2 * M_PI); // cnt/rad high
const double CNT_PER_RAD_LOW = RES_LOW / (2 * M_PI);           // cnt/rad low

int offset_cnt_high = 0; // 高速端cnt offset
int offset_cnt_low = 0;  // 低速端cnt offset

int isFirst = 0;
double pos = 0.0;
double last_pos = 0.0; // 上次的位置
double vel = 0.0;
double offset_torque = 0.0;
double last_offset_torque = 0.0; // 上次的力矩差
double vel_torque = 0.0;

double command_torque = 0.0;

//Iir::Butterworth::LowPass<2> filter; // 滤波器
const float sampleRate = 200;
const float cutoff_freq = 5; // default 5

const int inner_sign = 1;
const int outer_sign = 1;//根据内外圈的方向而定，如若外圈和内圈的方向一致，则为1，否则为-1，现默认方向一致调整为1  

double inner_rad = 0.0;
double outer_rad = 0.0;
double target_torque = 0.0;


////////////////////Extern Var///////////////////

int ctrl_mode = IMP_CTRL; //默认阻抗模式
int last_ctrl_mode = IMP_CTRL;
bool enabled = true;      //默认启动

double imp_stiff = 7.0;  // 默认阻抗刚度
double imp_damp  = 0.5;  // 默认阻抗阻尼

int g_status = STAT_STOPPED; //default stopped

//////////////////VAR DEFINE/////////////////////


////////////////FROM EcatConfig.h///////////////


// etherCAT object
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainTx = NULL;
static ec_domain_state_t domainTx_state = {};
static ec_domain_t *domainRx = NULL;
static ec_domain_state_t domainRx_state = {};

//==================== PDO =========================
static uint8_t *domainRx_pd = NULL;
static uint8_t *domainTx_pd = NULL;

// length change with number of slave(s)
static ec_slave_config_t *sc[joint_num];
static ec_slave_config_state_t sc_state[joint_num] = {};
static ec_sdo_request_t *sdo[joint_num];

// hardware specification
#define SynapticonSlave1 0, 0 // 1st slave
// #define SynapticonSlave2 0,1 // 2ed slave
// #define SynapticonSlave3 0,2 // 3rd slave
// #define SynapticonSlave4 0,3 // 1st slave
// #define SynapticonSlave5 0,4 // 2ed slave
// #define SynapticonSlave6 0,5 // 3rd slave

#define Synapticon 0x000022d2, 0x00000201 // vendor id + product id

typedef struct Joint_List
{
    int slave_id;
    unsigned char info[20];
} Joint_List;

// EtherCAT state machine enum
typedef enum _workingStatus
{
    sys_working_POWER_ON,
    sys_working_SAFE_MODE,
    sys_working_OP_MODE,
    sys_working_LINK_DOWN,
    sys_working_WORK_STATUS,
    sys_woring_INIT_Failed
} workingStatus;

typedef struct _GsysRunningParm
{
    workingStatus m_gWorkStatus;
} GsysRunningParm;

// OP task FSM
typedef enum _TaskFSM
{
    task_working_RESET,
    task_working_CALIBRATION,
    task_working_Identification,
    task_working_Impedance,
    task_working_CSP_tracking,
    task_working_Noload2Sit,
    task_working_Impedance_GRF,
    task_working_Transparency,
    task_working_Checking
} TaskFSM;

typedef struct _GTaskFSM
{
    TaskFSM m_gtaskFSM;
} GTaskFSM;

GsysRunningParm gSysRunning;
GTaskFSM gTaskFsm;

int SERVE_OP = 0;
int ecstate = 0;
int reset_step = 0;

// Offsets for PDO entries
static struct
{
    /* RxPDOs: 0x1600 */
    unsigned int operation_mode[joint_num];
    unsigned int ctrl_word[joint_num];
    unsigned int target_velocity[joint_num];
    unsigned int target_position[joint_num];
    unsigned int target_torque[joint_num];
    /* RxPDOs: 0x1601 */
    unsigned int digital_out1[joint_num];
    unsigned int digital_out2[joint_num];
    unsigned int digital_out3[joint_num];
    unsigned int digital_out4[joint_num];

    /* TxPDOs: 0x1A00 */
    unsigned int status_word[joint_num];
    unsigned int modes_of_operation_display[joint_num];
    unsigned int actual_position[joint_num];
    unsigned int actual_velocity[joint_num];
    unsigned int actual_torque[joint_num];
    /* TxPDOs: 0x1A01 */
    unsigned int second_position[joint_num];
    unsigned int second_velocity[joint_num];
    unsigned int analog_in1[joint_num];
    unsigned int analog_in2[joint_num];
    unsigned int analog_in3[joint_num];
    unsigned int analog_in4[joint_num];
    /* TxPDOs: 0x1A02 */
    unsigned int digital_in1[joint_num];
    unsigned int digital_in2[joint_num];
    unsigned int digital_in3[joint_num];
    unsigned int digital_in4[joint_num];
    /* Error Code */
    unsigned int Error_code[joint_num];
} offset;

/* NOTICE:
 * domain_reg must aligned to pdo_entries
 * the PDOs are configured to slaves not master
 *  slaves -> master = TxPOD
 *  master -> slaves = RxPOD
 */

// output_1 domain register (RxPDO mapped objects)
ec_pdo_entry_reg_t domain_Rx_reg[] = {
    // slave - 1
    {SynapticonSlave1, Synapticon, 0x6040, 0, &offset.ctrl_word[0]},
    {SynapticonSlave1, Synapticon, 0x6060, 0, &offset.operation_mode[0]},
    {SynapticonSlave1, Synapticon, 0x60FF, 0, &offset.target_velocity[0]},
    {SynapticonSlave1, Synapticon, 0x607A, 0, &offset.target_position[0]},
    {SynapticonSlave1, Synapticon, 0x6071, 0, &offset.target_torque[0]},
    //        // slave - 2
    //        {SynapticonSlave2, Synapticon, 0x6040, 0, &offset.ctrl_word[r_knee]},
    //        {SynapticonSlave2, Synapticon, 0x6060, 0, &offset.operation_mode[r_knee]},
    //        {SynapticonSlave2, Synapticon, 0x60FF, 0, &offset.target_velocity[r_knee]},
    //        {SynapticonSlave2, Synapticon, 0x607A, 0, &offset.target_position[r_knee]},
    //        {SynapticonSlave2, Synapticon, 0x6071, 0, &offset.target_torque[r_knee]},
    //        // slave - 3
    //        {SynapticonSlave3, Synapticon, 0x6040, 0, &offset.ctrl_word[r_ankle]},
    //        {SynapticonSlave3, Synapticon, 0x6060, 0, &offset.operation_mode[r_ankle]},
    //        {SynapticonSlave3, Synapticon, 0x60FF, 0, &offset.target_velocity[r_ankle]},
    //        {SynapticonSlave3, Synapticon, 0x607A, 0, &offset.target_position[r_ankle]},
    //        {SynapticonSlave3, Synapticon, 0x6071, 0, &offset.target_torque[r_ankle]},
    //        // slave - 4
    //        {SynapticonSlave4, Synapticon, 0x6040, 0, &offset.ctrl_word[l_hip]},
    //        {SynapticonSlave4, Synapticon, 0x6060, 0, &offset.operation_mode[l_hip]},
    //        {SynapticonSlave4, Synapticon, 0x60FF, 0, &offset.target_velocity[l_hip]},
    //        {SynapticonSlave4, Synapticon, 0x607A, 0, &offset.target_position[l_hip]},
    //        {SynapticonSlave4, Synapticon, 0x6071, 0, &offset.target_torque[l_hip]},
    //        // slave - 5
    //        {SynapticonSlave5, Synapticon, 0x6040, 0, &offset.ctrl_word[l_knee]},
    //        {SynapticonSlave5, Synapticon, 0x6060, 0, &offset.operation_mode[l_knee]},
    //        {SynapticonSlave5, Synapticon, 0x60FF, 0, &offset.target_velocity[l_knee]},
    //        {SynapticonSlave5, Synapticon, 0x607A, 0, &offset.target_position[l_knee]},
    //        {SynapticonSlave5, Synapticon, 0x6071, 0, &offset.target_torque[l_knee]},
    //        // slave - 6
    //        {SynapticonSlave6, Synapticon, 0x6040, 0, &offset.ctrl_word[l_ankle]},
    //        {SynapticonSlave6, Synapticon, 0x6060, 0, &offset.operation_mode[l_ankle]},
    //        {SynapticonSlave6, Synapticon, 0x60FF, 0, &offset.target_velocity[l_ankle]},
    //        {SynapticonSlave6, Synapticon, 0x607A, 0, &offset.target_position[l_ankle]},
    //        {SynapticonSlave6, Synapticon, 0x6071, 0, &offset.target_torque[l_ankle]},
    {}};

// input_1 domain register (TxPDO mapped objects)
ec_pdo_entry_reg_t domain_Tx_reg[] = {
    // slave - 1
    {SynapticonSlave1, Synapticon, 0x6041, 0, &offset.status_word[0]},
    {SynapticonSlave1, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[0]},
    {SynapticonSlave1, Synapticon, 0x6077, 0, &offset.actual_torque[0]},
    {SynapticonSlave1, Synapticon, 0x6064, 0, &offset.actual_position[0]},
    {SynapticonSlave1, Synapticon, 0x606C, 0, &offset.actual_velocity[0]},

    {SynapticonSlave1, Synapticon, 0x230A, 0, &offset.second_position[0]},
    {SynapticonSlave1, Synapticon, 0x230B, 0, &offset.second_velocity[0]},
    {SynapticonSlave1, Synapticon, 0x603F, 0, &offset.Error_code[0]},
    {}};

// PDO entries
static ec_pdo_entry_info_t pdo_entries_Rx[] = {
    /* RxPdo 0x1600 */
    {0x6040, 0x00, 16}, // control word
    {0x6060, 0x00, 8},  // Modes of operation
    {0x60FF, 0x00, 32}, // Target velocity
    {0x607A, 0x00, 32}, // target position
    {0x6071, 0x00, 16}, // Target torque
};

static ec_pdo_entry_info_t pdo_entries_Tx[] = {
    /* TxPdo 0x1A00 */
    {0x6041, 0x00, 16}, // Status word
    {0x6061, 0x00, 8},  // modes of operation display
    {0x606C, 0x00, 32}, // actual velocity
    {0x6064, 0x00, 32}, // actual position
    {0x6077, 0x00, 16}, // actual torque (unit is per thousand of rated torque.)
    /* TxPdo 0x1A01 */
    {0x230A, 0x00, 32}, // Second position
    {0x230B, 0x00, 32}, // Second velocity
    {0x2401, 0x00, 16}, // Analog Input 1
    {0x2402, 0x00, 16}, // Analog Input 2
    {0x603F, 0x00, 16}, // Error code
    {}};

// RxPDO
static ec_pdo_info_t RxPDOs[] = {
    /* RxPdo 0x1600 */
    {0x1600, 5, pdo_entries_Rx},
};

static ec_pdo_info_t TxPDOs[] = {
    /* TxPdo 0x1A00 */
    {0x1A00, 5, pdo_entries_Tx + 0},
    {0x1A01, 5, pdo_entries_Tx + 5}};

/*
 * output_1 = values written by master
 * input_1  = values written by slaves
 */
static ec_sync_info_t device_syncs[] = {
    //        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    //        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, RxPDOs, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, TxPDOs, EC_WD_DISABLE},
    {0xFF}};

void releaseMaster(void)
{
    if (master)
    {
        std::cout << std::endl;
        std::cout << "Released EtherCAT-Master Object." << std::endl;
        ecrt_release_master(master);
        master = NULL;
    }
}

int configPDO()
{
    std::cout << "Configuring PDOs ... " << std::endl;
    domainRx = ecrt_master_create_domain(master);
    if (!domainRx)
        exit(EXIT_FAILURE);
    domainTx = ecrt_master_create_domain(master);
    if (!domainTx)
        exit(EXIT_FAILURE);

    std::cout << "Creating slave configurations ... " << std::endl;

    /*
     * Obtain configuration of slaves
     */
    // slave 1
    if (!(sc[0] = ecrt_master_slave_config(master, SynapticonSlave1, Synapticon)))
    {
        std::cout << "Failed to get slave 1 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }
    //
    //    // slave 2
    //    if (!(sc[l_knee] = ecrt_master_slave_config(master, SynapticonSlave2, Synapticon))) {
    //        std::cout << "Failed to get slave 2 configuration. " << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //
    //    // slave 3
    //    if (!(sc[l_ankle] = ecrt_master_slave_config(master, SynapticonSlave3, Synapticon))) {
    //        std::cout << "Failed to get slave 3 configuration. " << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //    // slave 4
    //    if (!(sc[r_hip] = ecrt_master_slave_config(master, SynapticonSlave4, Synapticon))) {
    //        std::cout << "Failed to get slave 4 configuration. " << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //    // slave 5
    //    if (!(sc[r_knee] = ecrt_master_slave_config(master, SynapticonSlave5, Synapticon))) {
    //        std::cout << "Failed to get slave 5 configuration. " << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //    // slave 6
    //    if (!(sc[r_ankle] = ecrt_master_slave_config(master, SynapticonSlave6, Synapticon))) {
    //        std::cout << "Failed to get slave 6 configuration. " << std::endl;
    //        exit(EXIT_FAILURE);
    //    }

    /*
     * Configuring slaves' PDOs
     */
    // slave 1
    if (ecrt_slave_config_pdos(sc[0], EC_END, device_syncs))
    {
        std::cout << "Failed to config slave 1 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }
    //    // slave 2
    //    if (ecrt_slave_config_pdos(sc[r_knee], EC_END, device_syncs)) {
    //        std::cout << "Failed to config slave 2 PDOs" << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //    // slave 3
    //    if (ecrt_slave_config_pdos(sc[r_hip], EC_END, device_syncs)) {
    //        std::cout << "Failed to config slave 3 PDOs" << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //
    //    // slave 4
    //    if (ecrt_slave_config_pdos(sc[l_hip], EC_END, device_syncs)) {
    //        std::cout << "Failed to config slave 4 PDOs" << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //    // slave 5
    //    if (ecrt_slave_config_pdos(sc[l_knee], EC_END, device_syncs)) {
    //        std::cout << "Failed to config slave 5 PDOs" << std::endl;
    //        exit(EXIT_FAILURE);
    //    }
    //    // slave 6
    //    if (ecrt_slave_config_pdos(sc[l_ankle], EC_END, device_syncs)) {
    //        std::cout << "Failed to config slave 6 PDOs" << std::endl;
    //        exit(EXIT_FAILURE);
    //    }

    //    sdo[l_ankle] = ecrt_slave_config_create_sdo_request(sc[l_ankle],0x3102,2,2);

    if (ecrt_domain_reg_pdo_entry_list(domainRx, domain_Rx_reg))
    {
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    }
    if (ecrt_domain_reg_pdo_entry_list(domainTx, domain_Tx_reg))
    {
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    }

    return 0;
}

// =================== Function =======================

void check_domain_state(void)
{
    ec_domain_state_t ds = {};
    ec_domain_state_t ds1 = {};

    ecrt_domain_state(domainTx, &ds);
    if (ds.working_counter != domainTx_state.working_counter)
        std::cout << "domainTx: WC " << ds.working_counter << std::endl;
    if (ds.wc_state != domainTx_state.wc_state)
        std::cout << "domainTx: state " << ds.wc_state << std::endl;

    domainTx_state = ds;

    ecrt_domain_state(domainRx, &ds1);
    if (ds1.working_counter != domainRx_state.working_counter)
        std::cout << "domainRx: WC " << ds1.working_counter << std::endl;
    if (ds1.wc_state != domainRx_state.wc_state)
        std::cout << "domainRx: state " << ds1.wc_state << std::endl;

    domainRx_state = ds1;
}

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        std::cout << ms.slaves_responding << " slave(s)." << std::endl;
    if (ms.al_states != master_state.al_states)
        std::cout << "AL state : " << ms.al_states << std::endl;
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    master_state = ms;
}

void check_slave_config_states(void)
{
    ec_slave_config_state_t s[joint_num];

    for (int j = 0; j < joint_num; j++)
    {
        ecrt_slave_config_state(sc[j], &s[j]);
        if (s[j].al_state != sc_state[j].al_state)
            printf("Slave %d: State 0x%02X.\n", j, s[j].al_state);
        if (s[j].online != sc_state[j].online)
            printf("Slave %d: %s.\n", j, s[j].online ? "online" : "offline");
        if (s[j].operational != sc_state[j].operational)
            printf("slave %d: %soperational.\n", j, s[j].operational ? "" : "Not ");
        sc_state[j] = s[j];
    }
}

int ActivateMaster()
{
    std::cout << "Requesting master ... " << std::endl;

    if (master)
        return 0;

    master = ecrt_request_master(0);
    if (!master)
    {
        return -1;
    }

    configPDO();

    std::cout << "Activating master... " << std::endl;

    if (ecrt_master_activate(master))
    {
        exit((EXIT_FAILURE));
        std::cout << "Activating master...failed. " << std::endl;
    }

    if (!(domainTx_pd = ecrt_domain_data(domainTx)))
    {
        std::cout << "Failed to get domain data pointer. " << std::endl;
    }

    if (!(domainRx_pd = ecrt_domain_data(domainRx)))
    {
        std::cout << "Failed to get domain data pointer. " << std::endl;
    }

    std::cout << "Activating master...success. " << std::endl;

    return 0;
}

// =================== Thread   =======================

static void SIG_handle(int sig);

void cyclic_task(int task_Cmd);

int pause_to_continue()
{
    /**
     * Only Used For Special DEBUG.
     * ---- this function will interrupt EtherCAT communication loop
     */
    std::cout << "Pause Now." << std::endl;
    std::cout << "Press any key to continue..." << std::endl;
    struct termios tm, tm_old;
    int fd = STDIN_FILENO, c;
    if (tcgetattr(fd, &tm) < 0)
        return -1;
    tm_old = tm;
    cfmakeraw(&tm);
    if (tcsetattr(fd, TCSANOW, &tm) < 0)
        return -1;
    c = fgetc(stdin);

    if (tcsetattr(fd, TCSANOW, &tm_old) < 0)
        return -1;
    return c;
}

// ======== Some interface of EtherCAT ======
inline void SwitchUp_OP_mode(int slave_id, int Mode)
{
    int Current_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[slave_id]);
    if (Current_mode != Mode)
        EC_WRITE_S8(domainRx_pd + offset.operation_mode[slave_id], Mode);
}

void SwitchUp_OP_mode(int Mode)
{
    for (int i = 0; i < 6; i++)
        SwitchUp_OP_mode(i, Mode);
}


///////////////END FROM EcatConfig.h/////////////

// ============= FLG ===============
bool EtherCAT_ONLINE = true;

static void SIG_handle(int sig)
{
    if (sig == SIGINT)
    {
        EtherCAT_ONLINE = false;
        std::cout << "interrupt caught." << std::endl;
    }
}

void *robotcontrol(void *arg)
{

    gSysRunning.m_gWorkStatus = sys_working_POWER_ON;

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
    {
        ActivateMaster();
        ecstate = 0;
        gSysRunning.m_gWorkStatus = sys_working_SAFE_MODE;
        std::cout << "sys_working_SAFE_MODE." << std::endl;
    }

    ecrt_master_receive(master);

    while (EtherCAT_ONLINE)
    {
        signal(SIGINT, SIG_handle);

        usleep(1000);

        cyclic_task(0);
    }
    releaseMaster();

    pthread_exit(nullptr);
}

void cyclic_task(int task_Cmd)
{

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
        return;

    static int cycle_count = 0;
    cycle_count++;

    ecrt_master_receive(master);
    ecrt_domain_process(domainRx);
    ecrt_domain_process(domainTx);

    check_domain_state();

    if (!(cycle_count % 500))
    {
        check_master_state();
        check_slave_config_states();
    }

    switch (gSysRunning.m_gWorkStatus)
    {
    case sys_working_SAFE_MODE:
    {
        check_master_state();
        check_slave_config_states();

        if ((master_state.al_states & ETHERCAT_STATUS_OP))
        {
            bool tmp = true;

            for (int i = 0; i < active_num; i++)
            {
                if (!(sc_state[i].al_state & ETHERCAT_STATUS_OP))
                {
                    std::cout << "slave " << i << " al_state: " << sc_state[i].al_state << std::endl;
                    tmp = false;
                    gSysRunning.m_gWorkStatus = sys_woring_INIT_Failed;
                    break;
                }
            }

            if (tmp)
            {
                ecstate = 0;
                gSysRunning.m_gWorkStatus = sys_working_OP_MODE;
                std::cout << "[sys_working_OP_MODE]" << std::endl;
            }
        }
    }
    break;

    case sys_working_OP_MODE:
    {
        /**
         * Slaves Startup
         */
        ecstate++;
        if (SERVE_OP < active_num)
        {
            if (ecstate <= 10)
            {
                switch (ecstate)
                {
                case 1:
                    for (int i = 0; i < active_num; i++)
                    {
                        int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                        if (E_code != 0 || (EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x0008))
                        {
                            std::cout << "[Error: exception occur at slave: [" << i << ']' << std::endl;
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i],
                                         (EC_READ_U16(domainTx_pd + offset.status_word[i]) |
                                          0x0080));
                        } // bit7 set to 1 to reset fault
                        // EC_WRITE_U8(domainRx_pd + offset.operation_mode[i], CSV);
                        EC_WRITE_U8(domainRx_pd + offset.operation_mode[i], CST); // 切换到力矩模式
                    }
                    break;
                case 7:
                    //                            for (int i = 0; i < active_num; i++) {
                    //                                int cur_pos = EC_READ_S32(domainTx_pd + offset.actual_position[i]);
                    //                                EC_WRITE_S32(domainRx_pd + offset.target_position[i], cur_pos);
                    //                                std::cout << "Axis-" << i << " current position[cnt]: " << cur_pos << std::endl;
                    //                            }
                    break;
                default:
                    break;
                }
            }
            else
            {
                for (int i = 0; i < active_num; i++)
                {
                    unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                    if ((cur_status & 0x4f) == 0x40)
                        EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x06);
                    else if ((cur_status & 0x6f) == 0x21)
                        EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x07);
                    else if ((cur_status & 0x6f) == 0x23)
                    {
                        EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x0F);
                        SERVE_OP++;
                    }
                }
            }
        }
        else
        {
            int tmp = true;
            for (int i = 0; i < active_num; i++)
            {
                unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x6f) == 0x27)
                    std::cout << "Slave [" << i << "] Enable operation" << std::endl;
                else
                {
                    std::cout << "Slave [" << i << "] not in Enable operation" << std::endl;
                    std::cout << "Slave [" << i << "] State: " << cur_status << std::endl;
                    std::cout << "Slave [" << i << "] E-code: " << E_code << std::endl;
                }

                if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0)
                {
                    tmp = false;
                    ecstate = 0;
                    break;
                }
            }
            if (tmp)
            {
                ecstate = 0;
                gSysRunning.m_gWorkStatus = sys_working_WORK_STATUS;
                std::cout << "sys_working_WORK_STATUS" << std::endl;
            }
        }
    }
    break;

    default:
    {
        /** Preprocess of data */

        /**
         * Encoder cnt (Raw)
         */

        int cur_pos_high = EC_READ_S32(domainTx_pd + offset.actual_position[0]); // 高速端 pos(cnt)
        int cur_vel_high = EC_READ_S32(domainTx_pd + offset.actual_velocity[0]); // 高速端 vel(rpm?)
        int cur_tor_high = EC_READ_S16(domainTx_pd + offset.actual_torque[0]);   // 高速端 torque(1/1000 * 57)

        int cur_pos_low = EC_READ_S32(domainTx_pd + offset.second_position[0]); // 低速端 pos(cnt)
        int cur_vel_low = EC_READ_S32(domainTx_pd + offset.second_velocity[0]); // 低速端 vel(rpm?)

        EC_WRITE_S8(domainRx_pd + offset.operation_mode[0], CST); // 设置CST
        // EC_WRITE_S32(domainRx_pd+offset.target_velocity[0],100); // 发送目标速度函数

        if (isFirst < 5)
        {
            // 首次进入初始化
            offset_cnt_high = cur_pos_high; // 以刚启动的位置作为初始位置
            offset_cnt_low = cur_pos_low;

            // filter.setup(3, sampleRate, cutoff_freq);

            isFirst++; // 取消进入
        }

        if(last_ctrl_mode != ctrl_mode) {
            isFirst = 0;
            last_ctrl_mode = ctrl_mode;
        }

        ///////////////////////开始力环处理////////////////////////////
        if(abs(cur_vel_high) < 10)
            g_status = STAT_STOPPED;
        else
            g_status = STAT_RUNNING;

        inner_rad = inner_sign * (cur_pos_high - offset_cnt_high) / CNT_PER_RAD_HIGH; // 扭簧内圈弧度
        outer_rad = outer_sign * (cur_pos_low - offset_cnt_low) / CNT_PER_RAD_LOW;    // 扭簧外圈弧度

        double delta_rad = outer_rad - inner_rad;      // 内外圈弧度差值
        double sensor_torque = -delta_rad * STIFFNESS; // 检测到的外力矩

        pos = inner_rad; // 以外圈编码器作为最终位置
        vel = (pos - last_pos) / CYCLE_TIME;

        if(ctrl_mode == IMP_CTRL) {
            target_torque = -(pos * imp_stiff + vel * imp_damp); // 目标力矩
        }
        else if(ctrl_mode == ZERO_FORCE_CTRL) {
            target_torque = -sensor_torque;     // 目标力矩
        }

        // double torque_filter = filter.filter(sensor_torque); // 做一次滤波（SEA太稳定，不用滤波）

        offset_torque = target_torque - sensor_torque;
        vel_torque = (offset_torque - last_offset_torque);

        command_torque = target_torque + (15 * offset_torque + 3.5 * vel_torque);

        if (std::abs(command_torque) > 57)
        {
            command_torque = command_torque > 0 ? 57 : -57;
        }

        int send_torque_value = command_torque / 57.0 * 1000;

        if(!enabled)
            send_torque_value = 0;

        EC_WRITE_S16(domainRx_pd + offset.target_torque[0], send_torque_value); // 设置力矩
        // int vel = EC_READ_S32(domainTx_pd+offset.actual_velocity[0]);

        last_pos = pos;
        last_offset_torque = offset_torque;

        // EC_WRITE_S16(domainRx_pd + offset.target_torque[0], 100); // 设置力矩

        /** Recording initial cnt of each joint into csv-file */
//        if (!(cycle_count % 10))
//        {                    // show message per 10 ms. (base freq = 1Khz)
//            system("clear"); // 清屏
//            std::cout << "=====================" << std::endl;
//            std::cout << "task_working_Checking" << std::endl;
//            std::cout << "=====================" << std::endl;
//            std::cout << "HIGH>>>>>>>>>>>>>>>>>" << std::endl;
//            std::cout << "pos: " << cur_pos_high << " ; vel: " << cur_vel_high << " ; torque: " << cur_tor_high << " ; offset: " << offset_cnt_high << std::endl;

//            std::cout << "LOW>>>>>>>>>>>>>>>>>" << std::endl;
//            std::cout << "pos: " << cur_pos_low << " ; vel: " << cur_vel_low << " ; offset: " << offset_cnt_low << std::endl;

//            std::cout << "REAL>>>>>>>>>>>>>>>>" << std::endl;
//            std::cout << "inner: " << inner_rad << " ; outer: " << outer_rad << std::endl;

//            std::cout << "TORQUE>>>>>>>>>>>>>>>>" << std::endl;
//            std::cout << "target: " << target_torque << " ; sensor: " << sensor_torque << " ; command: " << command_torque << " ; send: " << send_torque_value << std::endl;
//            // std::cout << "target_torque: " << target_torque << " ; torque_filter: " << torque_filter << std::endl;

//            for (int i = 0; i < active_num; i++)
//            {
//                unsigned int error_code;
//                error_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);
//                if (error_code != 0x0000)
//                {
//                    std::cout << "=================================== " << std::endl;
//                    std::cout << "[" << i << "] slave reported error." << std::endl;
//                    std::cout << "----------------------------------- " << std::endl;
//                }
//            }
//        }
        //////////////END PRINT///////////////////
    }
    break;
    }

    // send process data objects
    ecrt_domain_queue(domainRx);
    ecrt_domain_queue(domainTx);
    ecrt_master_send(master);
}
