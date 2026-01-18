//
// Created by RM UI Designer
//

#include "ui_default_Ungroup_4.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 4
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Ungroup_4;
ui_interface_round_t *ui_default_Ungroup_NewRound = (ui_interface_round_t *)&(ui_default_Ungroup_4.data[0]);
ui_interface_round_t *ui_default_Ungroup_NewRound1 = (ui_interface_round_t *)&(ui_default_Ungroup_4.data[1]);
ui_interface_round_t *ui_default_Ungroup_NewRound2 = (ui_interface_round_t *)&(ui_default_Ungroup_4.data[2]);
ui_interface_line_t *ui_default_Ungroup_NewLine0 = (ui_interface_line_t *)&(ui_default_Ungroup_4.data[3]);
ui_interface_line_t *ui_default_Ungroup_NewLine1 = (ui_interface_line_t *)&(ui_default_Ungroup_4.data[4]);
ui_interface_line_t *ui_default_Ungroup_NewLine2 = (ui_interface_line_t *)&(ui_default_Ungroup_4.data[5]);
ui_interface_line_t *ui_default_Ungroup_NewLine3 = (ui_interface_line_t *)&(ui_default_Ungroup_4.data[6]);

void _ui_init_default_Ungroup_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_4.data[i].figure_name[0] = FRAME_ID;
        ui_default_Ungroup_4.data[i].figure_name[1] = GROUP_ID;
        ui_default_Ungroup_4.data[i].figure_name[2] = i + START_ID;
        ui_default_Ungroup_4.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Ungroup_4.data[i].operate_tpyel = 0;
    }

    ui_default_Ungroup_NewRound->figure_tpye = 2;
    ui_default_Ungroup_NewRound->layer = 0;
    ui_default_Ungroup_NewRound->r = 23;
    ui_default_Ungroup_NewRound->start_x = 1758;
    ui_default_Ungroup_NewRound->start_y = 752;
    ui_default_Ungroup_NewRound->color = 7;
    ui_default_Ungroup_NewRound->width = 10;

    ui_default_Ungroup_NewRound1->figure_tpye = 2;
    ui_default_Ungroup_NewRound1->layer = 0;
    ui_default_Ungroup_NewRound1->r = 23;
    ui_default_Ungroup_NewRound1->start_x = 1650;
    ui_default_Ungroup_NewRound1->start_y = 631;
    ui_default_Ungroup_NewRound1->color = 7;
    ui_default_Ungroup_NewRound1->width = 10;

    ui_default_Ungroup_NewRound2->figure_tpye = 2;
    ui_default_Ungroup_NewRound2->layer = 0;
    ui_default_Ungroup_NewRound2->r = 23;
    ui_default_Ungroup_NewRound2->start_x = 1698;
    ui_default_Ungroup_NewRound2->start_y = 432;
    ui_default_Ungroup_NewRound2->color = 7;
    ui_default_Ungroup_NewRound2->width = 10;

    ui_default_Ungroup_NewLine0->figure_tpye = 0;
    ui_default_Ungroup_NewLine0->layer = 1;
    ui_default_Ungroup_NewLine0->start_x = 850;
    ui_default_Ungroup_NewLine0->start_y = 393;
    ui_default_Ungroup_NewLine0->end_x = 1070;
    ui_default_Ungroup_NewLine0->end_y = 393;
    ui_default_Ungroup_NewLine0->color = 4;
    ui_default_Ungroup_NewLine0->width = 5;

    ui_default_Ungroup_NewLine1->figure_tpye = 0;
    ui_default_Ungroup_NewLine1->layer = 1;
    ui_default_Ungroup_NewLine1->start_x = 900;
    ui_default_Ungroup_NewLine1->start_y = 333;
    ui_default_Ungroup_NewLine1->end_x = 1019;
    ui_default_Ungroup_NewLine1->end_y = 333;
    ui_default_Ungroup_NewLine1->color = 4;
    ui_default_Ungroup_NewLine1->width = 3;

    ui_default_Ungroup_NewLine2->figure_tpye = 0;
    ui_default_Ungroup_NewLine2->layer = 1;
    ui_default_Ungroup_NewLine2->start_x = 959;
    ui_default_Ungroup_NewLine2->start_y = 202;
    ui_default_Ungroup_NewLine2->end_x = 959;
    ui_default_Ungroup_NewLine2->end_y = 880;
    ui_default_Ungroup_NewLine2->color = 4;
    ui_default_Ungroup_NewLine2->width = 5;

    ui_default_Ungroup_NewLine3->figure_tpye = 0;
    ui_default_Ungroup_NewLine3->layer = 1;
    ui_default_Ungroup_NewLine3->start_x = 900;
    ui_default_Ungroup_NewLine3->start_y = 272;
    ui_default_Ungroup_NewLine3->end_x = 1019;
    ui_default_Ungroup_NewLine3->end_y = 272;
    ui_default_Ungroup_NewLine3->color = 4;
    ui_default_Ungroup_NewLine3->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_4, sizeof(ui_default_Ungroup_4));
}

void _ui_update_default_Ungroup_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_4.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_4, sizeof(ui_default_Ungroup_4));
}

void _ui_remove_default_Ungroup_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Ungroup_4.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_default_Ungroup_4, sizeof(ui_default_Ungroup_4));
}
