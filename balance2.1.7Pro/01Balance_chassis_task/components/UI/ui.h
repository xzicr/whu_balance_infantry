//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif


// 图形配置参数：图形颜色
#define UI_Color_Main 0         // 红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 // 紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         // 青色
#define UI_Color_Black 7
#define UI_Color_White 8
#include "ui_interface.h"

#include "ui_default.h"

void ui_init_default_DynamicBottomGroup();
void ui_update_default_DynamicBottomGroup();
void ui_remove_default_DynamicBottomGroup();

void ui_init_default_DynamicLeftGroup();
void ui_update_default_DynamicLeftGroup();
void ui_remove_default_DynamicLeftGroup();

void ui_init_default_DynamicRightGroup();
void ui_update_default_DynamicRightGroup();
void ui_remove_default_DynamicRightGroup();

void ui_init_default_StaticGroup();
void ui_update_default_StaticGroup();
void ui_remove_default_StaticGroup();

void ui_init_default_StaticLeftGroup();
void ui_update_default_StaticLeftGroup();
void ui_remove_default_StaticLeftGroup();

void ui_init_default_StaticMiddleGroup();
void ui_update_default_StaticMiddleGroup();
void ui_remove_default_StaticMiddleGroup();
#include "ui_frame1.h"

void ui_init_frame1_DynamicNumberGroup();
void ui_update_frame1_DynamicNumberGroup();
void ui_remove_frame1_DynamicNumberGroup();

void ui_init_frame1_StaticNumberGroup();
void ui_update_frame1_StaticNumberGroup();
void ui_remove_frame1_StaticNumberGroup();

void ui_init_frame1_StaticTextGroup();
void ui_update_frame1_StaticTextGroup();
void ui_remove_frame1_StaticTextGroup();

#ifdef __cplusplus
}
#endif

#endif // UI_H
