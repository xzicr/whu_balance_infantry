//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_default_H
#define UI_default_H

#include "ui_interface.h"

extern ui_interface_round_t *ui_default_DynamicBottomGroup_FricRound;
extern ui_interface_round_t *ui_default_DynamicBottomGroup_AimRound;

void ui_init_default_DynamicBottomGroup();
void ui_update_default_DynamicBottomGroup();
void ui_remove_default_DynamicBottomGroup();

extern ui_interface_arc_t *ui_default_DynamicLeftGroup_ShootHeatArc;
extern ui_interface_arc_t *ui_default_DynamicLeftGroup_PowerArc;

void ui_init_default_DynamicLeftGroup();
void ui_update_default_DynamicLeftGroup();
void ui_remove_default_DynamicLeftGroup();

extern ui_interface_line_t *ui_default_DynamicHightGroup_HightLine;

void ui_init_default_DynamicHightGroup() ;
void ui_update_default_DynamicHightGroup() ;
void ui_remove_default_DynamicHightGroup() ;


extern ui_interface_arc_t *ui_default_DynamicRightGroup_DynamicPitchArc;
extern ui_interface_arc_t *ui_default_DynamicRightGroup_ChassisArc;

void ui_init_default_DynamicRightGroup();
void ui_update_default_DynamicRightGroup();
void ui_remove_default_DynamicRightGroup();

extern ui_interface_arc_t *ui_default_StaticGroup_PitchArc4;
extern ui_interface_line_t *ui_default_StaticGroup_GroundLineRight;
extern ui_interface_arc_t *ui_default_StaticGroup_PitchArc1;
extern ui_interface_arc_t *ui_default_StaticGroup_PitchArc2;
extern ui_interface_arc_t *ui_default_StaticGroup_PitchArc3;

void ui_init_default_StaticGroup();
void ui_update_default_StaticGroup();
void ui_remove_default_StaticGroup();

extern ui_interface_arc_t *ui_default_StaticLeftGroup_Arc1;
extern ui_interface_arc_t *ui_default_StaticLeftGroup_Arc2;
extern ui_interface_arc_t *ui_default_StaticLeftGroup_Arc3;
extern ui_interface_arc_t *ui_default_StaticLeftGroup_Arc4;
extern ui_interface_line_t *ui_default_StaticLeftGroup_GroundLineLeft;

void ui_init_default_StaticLeftGroup();
void ui_update_default_StaticLeftGroup();
void ui_remove_default_StaticLeftGroup();

extern ui_interface_line_t *ui_default_StaticMiddleGroup_CrossLine1;
extern ui_interface_line_t *ui_default_StaticMiddleGroup_CrossLine2;

void ui_init_default_StaticMiddleGroup();
void ui_update_default_StaticMiddleGroup();
void ui_remove_default_StaticMiddleGroup();


#endif // UI_default_H
