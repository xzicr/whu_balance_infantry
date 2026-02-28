//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_2_frame_t ui_default_DynamicBottomGroup_0;

ui_interface_round_t *ui_default_DynamicBottomGroup_FricRound = (ui_interface_round_t*)&(ui_default_DynamicBottomGroup_0.data[0]);
ui_interface_round_t *ui_default_DynamicBottomGroup_AimRound = (ui_interface_round_t*)&(ui_default_DynamicBottomGroup_0.data[1]);

void _ui_init_default_DynamicBottomGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicBottomGroup_0.data[i].figure_name[0] = 0;
        ui_default_DynamicBottomGroup_0.data[i].figure_name[1] = 0;
        ui_default_DynamicBottomGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_DynamicBottomGroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_default_DynamicBottomGroup_0.data[i].operate_type = 0;
    }

    ui_default_DynamicBottomGroup_FricRound->figure_type = 2;
    ui_default_DynamicBottomGroup_FricRound->operate_type = 1;
    ui_default_DynamicBottomGroup_FricRound->layer = 0;
    ui_default_DynamicBottomGroup_FricRound->color = 5;
    ui_default_DynamicBottomGroup_FricRound->start_x = 880;
    ui_default_DynamicBottomGroup_FricRound->start_y = 250;
    ui_default_DynamicBottomGroup_FricRound->width = 10;
    ui_default_DynamicBottomGroup_FricRound->r = 13;

    ui_default_DynamicBottomGroup_AimRound->figure_type = 2;
    ui_default_DynamicBottomGroup_AimRound->operate_type = 1;
    ui_default_DynamicBottomGroup_AimRound->layer = 0;
    ui_default_DynamicBottomGroup_AimRound->color = 8;
    ui_default_DynamicBottomGroup_AimRound->start_x = 960;
    ui_default_DynamicBottomGroup_AimRound->start_y = 250;
    ui_default_DynamicBottomGroup_AimRound->width = 10;
    ui_default_DynamicBottomGroup_AimRound->r = 13;


    ui_proc_2_frame(&ui_default_DynamicBottomGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicBottomGroup_0, sizeof(ui_default_DynamicBottomGroup_0));
}

void _ui_update_default_DynamicBottomGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicBottomGroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_default_DynamicBottomGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicBottomGroup_0, sizeof(ui_default_DynamicBottomGroup_0));
}

void _ui_remove_default_DynamicBottomGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicBottomGroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_default_DynamicBottomGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicBottomGroup_0, sizeof(ui_default_DynamicBottomGroup_0));
}


void ui_init_default_DynamicBottomGroup() {
    _ui_init_default_DynamicBottomGroup_0();
}

void ui_update_default_DynamicBottomGroup() {
    _ui_update_default_DynamicBottomGroup_0();
}

void ui_remove_default_DynamicBottomGroup() {
    _ui_remove_default_DynamicBottomGroup_0();
}

ui_2_frame_t ui_default_DynamicLeftGroup_0;

ui_interface_arc_t *ui_default_DynamicLeftGroup_ShootHeatArc = (ui_interface_arc_t*)&(ui_default_DynamicLeftGroup_0.data[0]);
ui_interface_arc_t *ui_default_DynamicLeftGroup_PowerArc = (ui_interface_arc_t*)&(ui_default_DynamicLeftGroup_0.data[1]);

void _ui_init_default_DynamicLeftGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicLeftGroup_0.data[i].figure_name[0] = 0;
        ui_default_DynamicLeftGroup_0.data[i].figure_name[1] = 1;
        ui_default_DynamicLeftGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_DynamicLeftGroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_default_DynamicLeftGroup_0.data[i].operate_type = 0;
    }

    ui_default_DynamicLeftGroup_ShootHeatArc->figure_type = 4;
    ui_default_DynamicLeftGroup_ShootHeatArc->operate_type = 1;
    ui_default_DynamicLeftGroup_ShootHeatArc->layer = 0;
    ui_default_DynamicLeftGroup_ShootHeatArc->color = 5;
    ui_default_DynamicLeftGroup_ShootHeatArc->start_x = 960;
    ui_default_DynamicLeftGroup_ShootHeatArc->start_y = 540;
    ui_default_DynamicLeftGroup_ShootHeatArc->width = 12;
    ui_default_DynamicLeftGroup_ShootHeatArc->start_angle = 250;
    ui_default_DynamicLeftGroup_ShootHeatArc->end_angle = 270;
    ui_default_DynamicLeftGroup_ShootHeatArc->rx = 390;
    ui_default_DynamicLeftGroup_ShootHeatArc->ry = 390;

    ui_default_DynamicLeftGroup_PowerArc->figure_type = 4;
    ui_default_DynamicLeftGroup_PowerArc->operate_type = 1;
    ui_default_DynamicLeftGroup_PowerArc->layer = 0;
    ui_default_DynamicLeftGroup_PowerArc->color = 3;
    ui_default_DynamicLeftGroup_PowerArc->start_x = 923;
    ui_default_DynamicLeftGroup_PowerArc->start_y = 540;
    ui_default_DynamicLeftGroup_PowerArc->width = 12;
    ui_default_DynamicLeftGroup_PowerArc->start_angle = 225;
    ui_default_DynamicLeftGroup_PowerArc->end_angle = 270;
    ui_default_DynamicLeftGroup_PowerArc->rx = 340;
    ui_default_DynamicLeftGroup_PowerArc->ry = 360;


    ui_proc_2_frame(&ui_default_DynamicLeftGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicLeftGroup_0, sizeof(ui_default_DynamicLeftGroup_0));
}

void _ui_update_default_DynamicLeftGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicLeftGroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_default_DynamicLeftGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicLeftGroup_0, sizeof(ui_default_DynamicLeftGroup_0));
}

void _ui_remove_default_DynamicLeftGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicLeftGroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_default_DynamicLeftGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicLeftGroup_0, sizeof(ui_default_DynamicLeftGroup_0));
}


void ui_init_default_DynamicLeftGroup() {
    _ui_init_default_DynamicLeftGroup_0();
}

void ui_update_default_DynamicLeftGroup() {
    _ui_update_default_DynamicLeftGroup_0();
}

void ui_remove_default_DynamicLeftGroup() {
    _ui_remove_default_DynamicLeftGroup_0();
}

ui_1_frame_t ui_default_DynamicHightGroup_0;

ui_interface_line_t *ui_default_DynamicHightGroup_HightLine = (ui_interface_line_t*)&(ui_default_DynamicHightGroup_0.data[0]);

void _ui_init_default_DynamicHightGroup_0() {
    for (int i = 0; i < 1; i++) {
        ui_default_DynamicHightGroup_0.data[i].figure_name[0] = 0;
        ui_default_DynamicHightGroup_0.data[i].figure_name[1] = 6;
        ui_default_DynamicHightGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_DynamicHightGroup_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_default_DynamicHightGroup_0.data[i].operate_type = 0;
    }

    ui_default_DynamicHightGroup_HightLine->figure_type = 0;
    ui_default_DynamicHightGroup_HightLine->operate_type = 1;
    ui_default_DynamicHightGroup_HightLine->layer = 0;
    ui_default_DynamicHightGroup_HightLine->color = 2;
    ui_default_DynamicHightGroup_HightLine->start_x = 160;
    ui_default_DynamicHightGroup_HightLine->start_y = 400;
    ui_default_DynamicHightGroup_HightLine->width = 20;
    ui_default_DynamicHightGroup_HightLine->end_x = 160;
    ui_default_DynamicHightGroup_HightLine->end_y = 680;




    ui_proc_1_frame(&ui_default_DynamicHightGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicHightGroup_0, sizeof(ui_default_DynamicHightGroup_0));
}

void _ui_update_default_DynamicHightGroup_0() {
    for (int i = 0; i < 1; i++) {
        ui_default_DynamicHightGroup_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_default_DynamicHightGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicHightGroup_0, sizeof(ui_default_DynamicHightGroup_0));
}

void _ui_remove_default_DynamicHightGroup_0() {
    for (int i = 0; i < 1; i++) {
        ui_default_DynamicHightGroup_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_default_DynamicHightGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicHightGroup_0, sizeof(ui_default_DynamicHightGroup_0));
}


void ui_init_default_DynamicHightGroup() {
    _ui_init_default_DynamicHightGroup_0();
}

void ui_update_default_DynamicHightGroup() {
    _ui_update_default_DynamicHightGroup_0();
}

void ui_remove_default_DynamicHightGroup() {
    _ui_remove_default_DynamicHightGroup_0();
}


ui_2_frame_t ui_default_DynamicRightGroup_0;

ui_interface_arc_t *ui_default_DynamicRightGroup_DynamicPitchArc = (ui_interface_arc_t*)&(ui_default_DynamicRightGroup_0.data[0]);
ui_interface_arc_t *ui_default_DynamicRightGroup_ChassisArc = (ui_interface_arc_t*)&(ui_default_DynamicRightGroup_0.data[1]);

void _ui_init_default_DynamicRightGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicRightGroup_0.data[i].figure_name[0] = 0;
        ui_default_DynamicRightGroup_0.data[i].figure_name[1] = 2;
        ui_default_DynamicRightGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_DynamicRightGroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_default_DynamicRightGroup_0.data[i].operate_type = 0;
    }

    ui_default_DynamicRightGroup_DynamicPitchArc->figure_type = 4;
    ui_default_DynamicRightGroup_DynamicPitchArc->operate_type = 1;
    ui_default_DynamicRightGroup_DynamicPitchArc->layer = 0;
    ui_default_DynamicRightGroup_DynamicPitchArc->color = 7;
    ui_default_DynamicRightGroup_DynamicPitchArc->start_x = 953;
    ui_default_DynamicRightGroup_DynamicPitchArc->start_y = 531;
    ui_default_DynamicRightGroup_DynamicPitchArc->width = 18;
    ui_default_DynamicRightGroup_DynamicPitchArc->start_angle = 90;
    ui_default_DynamicRightGroup_DynamicPitchArc->end_angle = 91;
    ui_default_DynamicRightGroup_DynamicPitchArc->rx = 380;
    ui_default_DynamicRightGroup_DynamicPitchArc->ry = 380;

    ui_default_DynamicRightGroup_ChassisArc->figure_type = 4;
    ui_default_DynamicRightGroup_ChassisArc->operate_type = 1;
    ui_default_DynamicRightGroup_ChassisArc->layer = 0;
    ui_default_DynamicRightGroup_ChassisArc->color = 6;
    ui_default_DynamicRightGroup_ChassisArc->start_x = 1630;
    ui_default_DynamicRightGroup_ChassisArc->start_y = 750;
    ui_default_DynamicRightGroup_ChassisArc->width = 10;
    ui_default_DynamicRightGroup_ChassisArc->start_angle = 200;
    ui_default_DynamicRightGroup_ChassisArc->end_angle = 160;
    ui_default_DynamicRightGroup_ChassisArc->rx = 50;
    ui_default_DynamicRightGroup_ChassisArc->ry = 50;


    ui_proc_2_frame(&ui_default_DynamicRightGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicRightGroup_0, sizeof(ui_default_DynamicRightGroup_0));
}

void _ui_update_default_DynamicRightGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicRightGroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_default_DynamicRightGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicRightGroup_0, sizeof(ui_default_DynamicRightGroup_0));
}

void _ui_remove_default_DynamicRightGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_DynamicRightGroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_default_DynamicRightGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_DynamicRightGroup_0, sizeof(ui_default_DynamicRightGroup_0));
}


void ui_init_default_DynamicRightGroup() {
    _ui_init_default_DynamicRightGroup_0();
}

void ui_update_default_DynamicRightGroup() {
    _ui_update_default_DynamicRightGroup_0();
}

void ui_remove_default_DynamicRightGroup() {
    _ui_remove_default_DynamicRightGroup_0();
}

ui_5_frame_t ui_default_StaticGroup_0;

ui_interface_arc_t *ui_default_StaticGroup_PitchArc4 = (ui_interface_arc_t*)&(ui_default_StaticGroup_0.data[0]);
ui_interface_line_t *ui_default_StaticGroup_GroundLineRight = (ui_interface_line_t*)&(ui_default_StaticGroup_0.data[1]);
ui_interface_arc_t *ui_default_StaticGroup_PitchArc1 = (ui_interface_arc_t*)&(ui_default_StaticGroup_0.data[2]);
ui_interface_arc_t *ui_default_StaticGroup_PitchArc2 = (ui_interface_arc_t*)&(ui_default_StaticGroup_0.data[3]);
ui_interface_arc_t *ui_default_StaticGroup_PitchArc3 = (ui_interface_arc_t*)&(ui_default_StaticGroup_0.data[4]);

void _ui_init_default_StaticGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_default_StaticGroup_0.data[i].figure_name[0] = 0;
        ui_default_StaticGroup_0.data[i].figure_name[1] = 3;
        ui_default_StaticGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_StaticGroup_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_default_StaticGroup_0.data[i].operate_type = 0;
    }

    ui_default_StaticGroup_PitchArc4->figure_type = 4;
    ui_default_StaticGroup_PitchArc4->operate_type = 1;
    ui_default_StaticGroup_PitchArc4->layer = 0;
    ui_default_StaticGroup_PitchArc4->color = 8;
    ui_default_StaticGroup_PitchArc4->start_x = 960;
    ui_default_StaticGroup_PitchArc4->start_y = 529;
    ui_default_StaticGroup_PitchArc4->width = 15;
    ui_default_StaticGroup_PitchArc4->start_angle = 70;
    ui_default_StaticGroup_PitchArc4->end_angle = 72;
    ui_default_StaticGroup_PitchArc4->rx = 400;
    ui_default_StaticGroup_PitchArc4->ry = 400;

    ui_default_StaticGroup_GroundLineRight->figure_type = 0;
    ui_default_StaticGroup_GroundLineRight->operate_type = 1;
    ui_default_StaticGroup_GroundLineRight->layer = 0;
    ui_default_StaticGroup_GroundLineRight->color = 1;
    ui_default_StaticGroup_GroundLineRight->start_x = 1368;
    ui_default_StaticGroup_GroundLineRight->start_y = 82;
    ui_default_StaticGroup_GroundLineRight->width = 3;
    ui_default_StaticGroup_GroundLineRight->end_x = 1254;
    ui_default_StaticGroup_GroundLineRight->end_y = 238;

    ui_default_StaticGroup_PitchArc1->figure_type = 4;
    ui_default_StaticGroup_PitchArc1->operate_type = 1;
    ui_default_StaticGroup_PitchArc1->layer = 0;
    ui_default_StaticGroup_PitchArc1->color = 8;
    ui_default_StaticGroup_PitchArc1->start_x = 960;
    ui_default_StaticGroup_PitchArc1->start_y = 529;
    ui_default_StaticGroup_PitchArc1->width = 20;
    ui_default_StaticGroup_PitchArc1->start_angle = 130;
    ui_default_StaticGroup_PitchArc1->end_angle = 132;
    ui_default_StaticGroup_PitchArc1->rx = 400;
    ui_default_StaticGroup_PitchArc1->ry = 400;

    ui_default_StaticGroup_PitchArc2->figure_type = 4;
    ui_default_StaticGroup_PitchArc2->operate_type = 1;
    ui_default_StaticGroup_PitchArc2->layer = 0;
    ui_default_StaticGroup_PitchArc2->color = 8;
    ui_default_StaticGroup_PitchArc2->start_x = 960;
    ui_default_StaticGroup_PitchArc2->start_y = 529;
    ui_default_StaticGroup_PitchArc2->width = 20;
    ui_default_StaticGroup_PitchArc2->start_angle = 50;
    ui_default_StaticGroup_PitchArc2->end_angle = 52;
    ui_default_StaticGroup_PitchArc2->rx = 400;
    ui_default_StaticGroup_PitchArc2->ry = 400;

    ui_default_StaticGroup_PitchArc3->figure_type = 4;
    ui_default_StaticGroup_PitchArc3->operate_type = 1;
    ui_default_StaticGroup_PitchArc3->layer = 0;
    ui_default_StaticGroup_PitchArc3->color = 8;
    ui_default_StaticGroup_PitchArc3->start_x = 960;
    ui_default_StaticGroup_PitchArc3->start_y = 529;
    ui_default_StaticGroup_PitchArc3->width = 15;
    ui_default_StaticGroup_PitchArc3->start_angle = 110;
    ui_default_StaticGroup_PitchArc3->end_angle = 112;
    ui_default_StaticGroup_PitchArc3->rx = 400;
    ui_default_StaticGroup_PitchArc3->ry = 400;


    ui_proc_5_frame(&ui_default_StaticGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticGroup_0, sizeof(ui_default_StaticGroup_0));
}

void _ui_update_default_StaticGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_default_StaticGroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_default_StaticGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticGroup_0, sizeof(ui_default_StaticGroup_0));
}

void _ui_remove_default_StaticGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_default_StaticGroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_default_StaticGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticGroup_0, sizeof(ui_default_StaticGroup_0));
}


void ui_init_default_StaticGroup() {
    _ui_init_default_StaticGroup_0();
}

void ui_update_default_StaticGroup() {
    _ui_update_default_StaticGroup_0();
}

void ui_remove_default_StaticGroup() {
    _ui_remove_default_StaticGroup_0();
}

ui_5_frame_t ui_default_StaticLeftGroup_0;

ui_interface_arc_t *ui_default_StaticLeftGroup_Arc1 = (ui_interface_arc_t*)&(ui_default_StaticLeftGroup_0.data[0]);
ui_interface_arc_t *ui_default_StaticLeftGroup_Arc2 = (ui_interface_arc_t*)&(ui_default_StaticLeftGroup_0.data[1]);
ui_interface_arc_t *ui_default_StaticLeftGroup_Arc3 = (ui_interface_arc_t*)&(ui_default_StaticLeftGroup_0.data[2]);
ui_interface_arc_t *ui_default_StaticLeftGroup_Arc4 = (ui_interface_arc_t*)&(ui_default_StaticLeftGroup_0.data[3]);
ui_interface_line_t *ui_default_StaticLeftGroup_GroundLineLeft = (ui_interface_line_t*)&(ui_default_StaticLeftGroup_0.data[4]);

void _ui_init_default_StaticLeftGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_default_StaticLeftGroup_0.data[i].figure_name[0] = 0;
        ui_default_StaticLeftGroup_0.data[i].figure_name[1] = 4;
        ui_default_StaticLeftGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_StaticLeftGroup_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_default_StaticLeftGroup_0.data[i].operate_type = 0;
    }

    ui_default_StaticLeftGroup_Arc1->figure_type = 4;
    ui_default_StaticLeftGroup_Arc1->operate_type = 1;
    ui_default_StaticLeftGroup_Arc1->layer = 0;
    ui_default_StaticLeftGroup_Arc1->color = 8;
    ui_default_StaticLeftGroup_Arc1->start_x = 923;
    ui_default_StaticLeftGroup_Arc1->start_y = 540;
    ui_default_StaticLeftGroup_Arc1->width = 15;
    ui_default_StaticLeftGroup_Arc1->start_angle = 225;
    ui_default_StaticLeftGroup_Arc1->end_angle = 226;
    ui_default_StaticLeftGroup_Arc1->rx = 340;
    ui_default_StaticLeftGroup_Arc1->ry = 360;

    ui_default_StaticLeftGroup_Arc2->figure_type = 4;
    ui_default_StaticLeftGroup_Arc2->operate_type = 1;
    ui_default_StaticLeftGroup_Arc2->layer = 0;
    ui_default_StaticLeftGroup_Arc2->color = 8;
    ui_default_StaticLeftGroup_Arc2->start_x = 923;
    ui_default_StaticLeftGroup_Arc2->start_y = 540;
    ui_default_StaticLeftGroup_Arc2->width = 15;
    ui_default_StaticLeftGroup_Arc2->start_angle = 270;
    ui_default_StaticLeftGroup_Arc2->end_angle = 271;
    ui_default_StaticLeftGroup_Arc2->rx = 340;
    ui_default_StaticLeftGroup_Arc2->ry = 360;

    ui_default_StaticLeftGroup_Arc3->figure_type = 4;
    ui_default_StaticLeftGroup_Arc3->operate_type = 1;
    ui_default_StaticLeftGroup_Arc3->layer = 0;
    ui_default_StaticLeftGroup_Arc3->color = 8;
    ui_default_StaticLeftGroup_Arc3->start_x = 960;
    ui_default_StaticLeftGroup_Arc3->start_y = 540;
    ui_default_StaticLeftGroup_Arc3->width = 15;
    ui_default_StaticLeftGroup_Arc3->start_angle = 250;
    ui_default_StaticLeftGroup_Arc3->end_angle = 251;
    ui_default_StaticLeftGroup_Arc3->rx = 390;
    ui_default_StaticLeftGroup_Arc3->ry = 390;

    ui_default_StaticLeftGroup_Arc4->figure_type = 4;
    ui_default_StaticLeftGroup_Arc4->operate_type = 1;
    ui_default_StaticLeftGroup_Arc4->layer = 0;
    ui_default_StaticLeftGroup_Arc4->color = 8;
    ui_default_StaticLeftGroup_Arc4->start_x = 960;
    ui_default_StaticLeftGroup_Arc4->start_y = 540;
    ui_default_StaticLeftGroup_Arc4->width = 15;
    ui_default_StaticLeftGroup_Arc4->start_angle = 270;
    ui_default_StaticLeftGroup_Arc4->end_angle = 271;
    ui_default_StaticLeftGroup_Arc4->rx = 390;
    ui_default_StaticLeftGroup_Arc4->ry = 390;

    ui_default_StaticLeftGroup_GroundLineLeft->figure_type = 0;
    ui_default_StaticLeftGroup_GroundLineLeft->operate_type = 1;
    ui_default_StaticLeftGroup_GroundLineLeft->layer = 0;
    ui_default_StaticLeftGroup_GroundLineLeft->color = 1;
    ui_default_StaticLeftGroup_GroundLineLeft->start_x = 552;
    ui_default_StaticLeftGroup_GroundLineLeft->start_y = 82;
    ui_default_StaticLeftGroup_GroundLineLeft->width = 3;
    ui_default_StaticLeftGroup_GroundLineLeft->end_x = 666;
    ui_default_StaticLeftGroup_GroundLineLeft->end_y = 238;


    ui_proc_5_frame(&ui_default_StaticLeftGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticLeftGroup_0, sizeof(ui_default_StaticLeftGroup_0));
}

void _ui_update_default_StaticLeftGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_default_StaticLeftGroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_default_StaticLeftGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticLeftGroup_0, sizeof(ui_default_StaticLeftGroup_0));
}

void _ui_remove_default_StaticLeftGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_default_StaticLeftGroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_default_StaticLeftGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticLeftGroup_0, sizeof(ui_default_StaticLeftGroup_0));
}


void ui_init_default_StaticLeftGroup() {
    _ui_init_default_StaticLeftGroup_0();
}

void ui_update_default_StaticLeftGroup() {
    _ui_update_default_StaticLeftGroup_0();
}

void ui_remove_default_StaticLeftGroup() {
    _ui_remove_default_StaticLeftGroup_0();
}

ui_2_frame_t ui_default_StaticMiddleGroup_0;

ui_interface_line_t *ui_default_StaticMiddleGroup_CrossLine1 = (ui_interface_line_t*)&(ui_default_StaticMiddleGroup_0.data[0]);
ui_interface_line_t *ui_default_StaticMiddleGroup_CrossLine2 = (ui_interface_line_t*)&(ui_default_StaticMiddleGroup_0.data[1]);

void _ui_init_default_StaticMiddleGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_StaticMiddleGroup_0.data[i].figure_name[0] = 0;
        ui_default_StaticMiddleGroup_0.data[i].figure_name[1] = 5;
        ui_default_StaticMiddleGroup_0.data[i].figure_name[2] = i + 0;
        ui_default_StaticMiddleGroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_default_StaticMiddleGroup_0.data[i].operate_type = 0;
    }

    ui_default_StaticMiddleGroup_CrossLine1->figure_type = 0;
    ui_default_StaticMiddleGroup_CrossLine1->operate_type = 1;
    ui_default_StaticMiddleGroup_CrossLine1->layer = 0;
    ui_default_StaticMiddleGroup_CrossLine1->color = 8;
    ui_default_StaticMiddleGroup_CrossLine1->start_x = 880;
    ui_default_StaticMiddleGroup_CrossLine1->start_y = 500;
    ui_default_StaticMiddleGroup_CrossLine1->width = 1;
    ui_default_StaticMiddleGroup_CrossLine1->end_x = 1039;
    ui_default_StaticMiddleGroup_CrossLine1->end_y = 500;

    ui_default_StaticMiddleGroup_CrossLine2->figure_type = 0;
    ui_default_StaticMiddleGroup_CrossLine2->operate_type = 1;
    ui_default_StaticMiddleGroup_CrossLine2->layer = 0;
    ui_default_StaticMiddleGroup_CrossLine2->color = 8;
    ui_default_StaticMiddleGroup_CrossLine2->start_x = 960;
    ui_default_StaticMiddleGroup_CrossLine2->start_y = 360;
    ui_default_StaticMiddleGroup_CrossLine2->width = 2;
    ui_default_StaticMiddleGroup_CrossLine2->end_x = 960;
    ui_default_StaticMiddleGroup_CrossLine2->end_y = 682;


    ui_proc_2_frame(&ui_default_StaticMiddleGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticMiddleGroup_0, sizeof(ui_default_StaticMiddleGroup_0));
}

void _ui_update_default_StaticMiddleGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_StaticMiddleGroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_default_StaticMiddleGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticMiddleGroup_0, sizeof(ui_default_StaticMiddleGroup_0));
}

void _ui_remove_default_StaticMiddleGroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_default_StaticMiddleGroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_default_StaticMiddleGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_default_StaticMiddleGroup_0, sizeof(ui_default_StaticMiddleGroup_0));
}


void ui_init_default_StaticMiddleGroup() {
    _ui_init_default_StaticMiddleGroup_0();
}

void ui_update_default_StaticMiddleGroup() {
    _ui_update_default_StaticMiddleGroup_0();
}

void ui_remove_default_StaticMiddleGroup() {
    _ui_remove_default_StaticMiddleGroup_0();
}

