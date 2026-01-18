//
// Created by RM UI Designer
//

#include "ui_default_Ungroup_5.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 11
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Ungroup_5;
ui_interface_number_t *ui_default_Ungroup_NewFloat = (ui_interface_number_t *)&(ui_default_Ungroup_5.data[0]);

void _ui_init_default_Ungroup_5() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_5.data[i].figure_name[0] = FRAME_ID;
        ui_default_Ungroup_5.data[i].figure_name[1] = GROUP_ID;
        ui_default_Ungroup_5.data[i].figure_name[2] = i + START_ID;
        ui_default_Ungroup_5.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Ungroup_5.data[i].operate_tpyel = 0;
    }

    ui_default_Ungroup_NewFloat->figure_tpye = 5;
    ui_default_Ungroup_NewFloat->layer = 0;
    ui_default_Ungroup_NewFloat->font_size = 20;
    ui_default_Ungroup_NewFloat->start_x = 1599;
    ui_default_Ungroup_NewFloat->start_y = 845;
    ui_default_Ungroup_NewFloat->color = 7;
    ui_default_Ungroup_NewFloat->number = 12345;
    ui_default_Ungroup_NewFloat->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_5, sizeof(ui_default_Ungroup_5));
}

void _ui_update_default_Ungroup_5() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_5.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_5, sizeof(ui_default_Ungroup_5));
}

void _ui_remove_default_Ungroup_5() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_5.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_5, sizeof(ui_default_Ungroup_5));
}
