/**
 * @file        judge_task.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        19-July-2021
 * @brief       Judge Center
 */

/* Includes ------------------------------------------------------------------*/
#include "judge_task.h"

#include "judge_sensor.h"
#include "gimbal.h"
#include "rc_sensor.h"
#include "cmsis_os.h"
#include "string.h"

extern float vis_offset_yaw;
extern float vis_offset_pit;
/* Private macro -------------------------------------------------------------*/
#define STR_FONT_SIZE   (20)
#define STR_FONT_WIDTH  (2)
#define LINE_WIDTH      (2)
// yaw:
#define STR_YAW_X       (280)
#define STR_YAW_Y       (660)
// pit:
#define STR_PIT_X       (280)
#define STR_PIT_Y       (600)
// yawÆ«ÒÆ
#define INT_YAW_X       (STR_YAW_X + 5*20)
#define INT_YAW_Y       (STR_YAW_Y)
// pitÆ«ÒÆ
#define INT_PIT_X       (STR_PIT_X + 5*20)
#define INT_PIT_Y       (STR_PIT_Y)
// 30sµ¹¼ÆÊ±
#define INT_30s_X       (960-10)
#define INT_30s_Y       (540+200)
// µ¯µÀ¸¨ÖúÏß1
#define LINE_1_X0       (960-20)
#define LINE_1_X1       (960-40)
#define LINE_1_Y0       (540-70)
#define LINE_1_Y1       (540+60)
// µ¯µÀ¸¨ÖúÏß2
#define LINE_2_X0       (960+20)
#define LINE_2_X1       (960+40)
#define LINE_2_Y0       (540-70)
#define LINE_2_Y1       (540+60)

/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static void config_graphic_name(graphic_data_struct_t *ui, const char *name)
{
    if(strlen(name) < 3)
        return;
    
    for(uint8_t i = 0; i < 3; i++)
        ui->graphic_name[i] = name[i];
}

/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void control_sentry(void)
{
    /*  °´ÏÂCtrl¼üÖ®ºó¿ÉÒÔ¿ØÖÆÉÚ±ø */
    if(rc_sensor.info->CTRL.state == PRESS) {
        /* Ctrl+A ¿ØÖÆÉÚ±øÍù×óÒÆ¶¯ */
        if(rc_sensor.info->A.state == PRESS) {
            judge_info.aerial_to_sentry.cmd |= ctrl_sentry; // ½Ó¹ÜÉÚ±ø
            //judge_info.aerial_to_sentry.cmd &= ~check_outpost; // ´ò¶ÏÊ©·¨(Í£Ö¹²é¿´Ç°ÉÚÕ¾)
            judge_info.aerial_to_sentry.ctrl_dir = -1;
        }
        /* Ctrl+D ¿ØÖÆÉÚ±RELEASE_TO_PRESS){
øÍùÓÒÒÆ¶¯ */
        else if(rc_sensor.info->D.state == PRESS) {
            judge_info.aerial_to_sentry.cmd |= ctrl_sentry; // ½Ó¹ÜÉÚ±ø
            judge_info.aerial_to_sentry.cmd &= ~check_road; // ´ò¶ÏÊ©·¨(Í£Ö¹²é¿´µÐ·½·ÉÆÂ¹«Â·)
            judge_info.aerial_to_sentry.ctrl_dir = +1;
        }
        /* Ctrl ÉÚ±ø×ÔÓÉÒÆ¶¯(²»Ó°ÏìÉÚ±øÔË¶¯) */
        else {
            judge_info.aerial_to_sentry.cmd &= ~ctrl_sentry;// Í£Ö¹½Ó¹ÜÉÚ±ø 
            judge_info.aerial_to_sentry.ctrl_dir = 0;
        }
        
        /* ÍùµÐ·½·ÉÆÂ¹«Â·¶ËÒÆ¶¯ */
        if(rc_sensor.info->Q.state == PRESS) {
            judge_info.aerial_to_sentry.cmd |= check_road;
        }
        else {
            /* ËÉ¿ªQ¼üÖ®ºóÎ¬³Ö6s²é¿´¹«Â· */
            if(judge_info.aerial_to_sentry.cmd & check_road) {
                if(rc_sensor.info->Q.hold_time >= 6000)
                    judge_info.aerial_to_sentry.cmd &= ~check_road;
            }
        }
        
        /* ²âÊÔÉÚ±ø */
        if(rc_sensor.info->E.state == PRESS) {
            /* ·ÀÖ¹Îó´¥ */
            if(rc_sensor.info->E.hold_time >= 100) {
                judge_info.aerial_to_sentry.cmd |= test_sentry;
            }
        } else {
            judge_info.aerial_to_sentry.cmd &= ~test_sentry;
        }
        
        /* Ctrl+W ÉÚ±øÉÏÏÂÔÆÌ¨Í£Ö¹¿ª»ð */
        if(rc_sensor.info->W.state == PRESS) {
            /* ·ÀÖ¹Îó´¥ */
            if(rc_sensor.info->W.hold_time >= 100) {
                judge_info.aerial_to_sentry.cmd |= stop_fire;
            }
        } else {
            judge_info.aerial_to_sentry.cmd &= ~stop_fire;
        }
       
        /* Ctrl+S ÉÚ±øÏÂÔÆÌ¨ÏòºóÉ¨Ãè */
        if(rc_sensor.info->S.state == PRESS) {
            /* ·ÀÖ¹Îó´¥ */
            if(rc_sensor.info->S.hold_time >= 100) {
                judge_info.aerial_to_sentry.cmd |= back_scan;
            }
        } else {
            judge_info.aerial_to_sentry.cmd &= ~back_scan;
        }
        
        /* Ctrl+R ÉÚ±øÇÐ»»ÅÜ¹ìÄ£Ê½*/
        if(rc_sensor.info->R.state == PRESS) {
            /* ·ÀÖ¹Îó´¥ */
            if(rc_sensor.info->R.hold_time >= 10) {
                judge_info.aerial_to_sentry.cmd |= escape;
            }
        } else {
            judge_info.aerial_to_sentry.cmd &= ~escape;
        }

        judge_sensor.aerial_to_sentry(&judge_sensor);
    }
    else {
        if(rc_sensor.info->CTRL.hold_time <= 500) {
            // ÃüÁîÂëÇåÁã
            judge_info.aerial_to_sentry.cmd = 0x00;
            judge_info.aerial_to_sentry.ctrl_dir = 0x00;
            judge_sensor.aerial_to_sentry(&judge_sensor);
        }
    }
}

static void control_radar(void)
{
    static button_state_t last_state_A = RELEASE;
    static button_state_t last_state_B = RELEASE;
    if(rc_sensor.info->A.state == PRESS && last_state_A == RELEASE){
        last_state_A = PRESS;
        judge_info.aerial_to_radar.data_cmd_id = 0x0206;
        judge_sensor.aerial_to_radar(&judge_sensor);
    }
    else if(rc_sensor.info->B.state == PRESS && last_state_B == RELEASE){
        last_state_B = PRESS;
        judge_info.aerial_to_radar.data_cmd_id = 0x0207;
        judge_sensor.aerial_to_radar(&judge_sensor);
    }
    if(rc_sensor.info->A.state == RELEASE)last_state_A = RELEASE;
    if(rc_sensor.info->B.state == RELEASE)last_state_B = RELEASE;  
}

static void client_init_ui(void)
{
    const char *str_yaw = "yaw:";
    config_graphic_name(&judge_sensor.info->aerial_ui_str_yaw.grapic_data_struct, "L00");
    judge_sensor.config_str(&judge_sensor.info->aerial_ui_str_yaw,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_0,
                            GRAPHIC_YELLOW,
                            STR_FONT_SIZE,
                            strlen(str_yaw),
                            STR_FONT_WIDTH,
                            STR_YAW_X,
                            STR_YAW_Y,
                            (uint8_t*)str_yaw);
    //judge_sensor.draw_str(&judge_sensor, &judge_sensor.info->aerial_ui_str_yaw);
    
    const char *str_pit = "pit:";
    config_graphic_name(&judge_sensor.info->aerial_ui_str_pit.grapic_data_struct, "L01");
    judge_sensor.config_str(&judge_sensor.info->aerial_ui_str_pit,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_0,
                            GRAPHIC_YELLOW,
                            STR_FONT_SIZE,
                            strlen(str_pit),
                            STR_FONT_WIDTH,
                            STR_PIT_X,
                            STR_PIT_Y,
                            (uint8_t*)str_pit);
    //judge_sensor.draw_str(&judge_sensor, &judge_sensor.info->aerial_ui_str_pit);
    
    config_graphic_name(&judge_sensor.info->aerial_ui_int_yaw.grapic_data_struct, "L10");
    judge_sensor.config_int(&judge_sensor.info->aerial_ui_int_yaw.grapic_data_struct,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_1,
                            GRAPHIC_GREEN,
                            STR_FONT_SIZE,
                            STR_FONT_WIDTH,
                            INT_YAW_X,
                            INT_YAW_Y,
                            (int32_t)vis_offset_yaw);
    //judge_sensor.draw_1_fig(&judge_sensor, &judge_sensor.info->aerial_ui_int_yaw);
                      
    config_graphic_name(&judge_sensor.info->aerial_ui_int_pit.grapic_data_struct, "L11");
    judge_sensor.config_int(&judge_sensor.info->aerial_ui_int_pit.grapic_data_struct,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_1,
                            GRAPHIC_GREEN,
                            STR_FONT_SIZE,
                            STR_FONT_WIDTH,
                            INT_PIT_X,
                            INT_PIT_Y,
                            (int32_t)vis_offset_pit);
    //judge_sensor.draw_1_fig(&judge_sensor, &judge_sensor.info->aerial_ui_int_pit);

    config_graphic_name(&judge_sensor.info->aerial_ui_int_30s.grapic_data_struct, "L12");
    judge_sensor.config_int(&judge_sensor.info->aerial_ui_int_30s.grapic_data_struct,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_1,
                            GRAPHIC_FUCHSIA,
                            STR_FONT_SIZE,
                            STR_FONT_WIDTH,
                            INT_30s_X,
                            INT_30s_Y,
                            (int32_t)judge_sensor.info->aerial_robot_energy.attack_time);

    config_graphic_name(&judge_sensor.info->aerial_ui_line_1.grapic_data_struct, "L20");
    judge_sensor.config_line(&judge_sensor.info->aerial_ui_line_1.grapic_data_struct,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_2,
                            GRAPHIC_YELLOW,
                            LINE_WIDTH,
                            LINE_1_X0,
                            LINE_1_Y0,
                            LINE_1_X1,
                            LINE_1_Y1);
                 
    config_graphic_name(&judge_sensor.info->aerial_ui_line_2.grapic_data_struct, "L21");
    judge_sensor.config_line(&judge_sensor.info->aerial_ui_line_2.grapic_data_struct,
                            GRAPHIC_ADD,
                            GRAPHIC_LAYER_2,
                            GRAPHIC_YELLOW,
                            LINE_WIDTH,
                            LINE_2_X0,
                            LINE_2_Y0,
                            LINE_2_X1,
                            LINE_2_Y1);                            
                            
//    judge_sensor.client_ui_str_yaw(&judge_sensor, GRAPHIC_ADD);
//    judge_sensor.client_ui_str_pit(&judge_sensor, GRAPHIC_ADD);
//    judge_sensor.client_ui_int_yaw(&judge_sensor, GRAPHIC_ADD, (int32_t)vis_offset_yaw);
//    judge_sensor.client_ui_int_pit(&judge_sensor, GRAPHIC_ADD, (int32_t)vis_offset_pit);
}

button_t MOUSE_R;
button_flip_t MOUSE_R_TRIG;
uint8_t init_ui_flag = 0;
static void client_ui(void)
{
    static uint8_t init_step = 0;
    static uint8_t update_step = 0;
    MOUSE_R_TRIG = rc_sensor.copy_button(&MOUSE_R, &rc_sensor.info->MOUSE_R);
    
    if(MOUSE_R_TRIG == RELEASE_TO_PRESS) {
        switch(init_step)
        {
            case 0: {
                judge_sensor.info->aerial_ui_str_yaw.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.draw_str(&judge_sensor, &judge_sensor.info->aerial_ui_str_yaw);
                init_step = 1;
                break;
            }
            
            case 1: {
                judge_sensor.info->aerial_ui_str_pit.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.draw_str(&judge_sensor, &judge_sensor.info->aerial_ui_str_pit);
                init_step = 2;
                break;
            }
            
            case 2: {
                judge_sensor.info->aerial_ui_int_yaw.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.info->aerial_ui_int_pit.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.info->aerial_ui_int_30s.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.info->aerial_ui_line_1.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.info->aerial_ui_line_2.grapic_data_struct.operate_type = GRAPHIC_ADD;
                judge_sensor.draw_5_figs(&judge_sensor,
                                         &judge_sensor.info->aerial_ui_int_yaw,
                                         &judge_sensor.info->aerial_ui_int_pit,
                                         &judge_sensor.info->aerial_ui_int_30s,
                                         &judge_sensor.info->aerial_ui_line_1,
                                         &judge_sensor.info->aerial_ui_line_2);
                init_step = 0;
                break;
            }
            
            default: {
                init_step = 0;
                break;
            }
        }
    }
    else {
        judge_sensor.modify_int(&judge_sensor.info->aerial_ui_int_yaw.grapic_data_struct, (int32_t)vis_offset_yaw);
        judge_sensor.modify_int(&judge_sensor.info->aerial_ui_int_pit.grapic_data_struct, (int32_t)vis_offset_pit);
        judge_sensor.modify_int(&judge_sensor.info->aerial_ui_int_30s.grapic_data_struct, (int32_t)judge_sensor.info->aerial_robot_energy.attack_time);
        switch(update_step)
        {
            case 0: {
                judge_sensor.draw_2_figs(&judge_sensor,
                                         &judge_sensor.info->aerial_ui_int_30s,
                                         &judge_sensor.info->aerial_ui_int_yaw);
                update_step = 1;
                break;
            }

            case 1: {
                judge_sensor.draw_2_figs(&judge_sensor,
                         &judge_sensor.info->aerial_ui_int_30s,
                         &judge_sensor.info->aerial_ui_int_pit);
                update_step = 0;
                break;
            }

            default: {
                update_step = 0;
                break;
            } 
        }
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	Êý¾ÝÏûÏ¢´¦ÀíÈÎÎñ
 */
extern osThreadId JudgeTaskHandle;
void StartJudgeTask(void const * argument)
{
    client_init_ui();
	for(;;)
	{
        control_sentry();
        control_radar();
        client_ui();
        
		osDelay(100);
	}
}
