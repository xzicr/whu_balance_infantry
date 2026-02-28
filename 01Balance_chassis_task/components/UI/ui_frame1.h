//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_frame1_H
#define UI_frame1_H

#include "ui_interface.h"

extern ui_interface_number_t *ui_frame1_DynamicNumberGroup_NewNumber;

void ui_init_frame1_DynamicNumberGroup();
void ui_update_frame1_DynamicNumberGroup();
void ui_remove_frame1_DynamicNumberGroup();

extern ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber1;
extern ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber2;
extern ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber3;
extern ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber4;
extern ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber5;

void ui_init_frame1_StaticNumberGroup();
void ui_update_frame1_StaticNumberGroup();
void ui_remove_frame1_StaticNumberGroup();

extern ui_interface_string_t *ui_frame1_StaticTextGroup_HeatText;
extern ui_interface_string_t *ui_frame1_StaticTextGroup_PowerText;
extern ui_interface_string_t *ui_frame1_StaticTextGroup_FricText;
extern ui_interface_string_t *ui_frame1_StaticTextGroup_AutoText;
extern ui_interface_string_t *ui_frame1_StaticTextGroup_Text1;

void ui_init_frame1_StaticTextGroup();
void ui_update_frame1_StaticTextGroup();
void ui_remove_frame1_StaticTextGroup();


#endif // UI_frame1_H
