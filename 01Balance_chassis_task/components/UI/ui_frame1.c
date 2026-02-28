//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_1_frame_t ui_frame1_DynamicNumberGroup_0;

ui_interface_number_t *ui_frame1_DynamicNumberGroup_NewNumber = (ui_interface_number_t*)&(ui_frame1_DynamicNumberGroup_0.data[0]);

void _ui_init_frame1_DynamicNumberGroup_0() {
    for (int i = 0; i < 1; i++) {
        ui_frame1_DynamicNumberGroup_0.data[i].figure_name[0] = 1;
        ui_frame1_DynamicNumberGroup_0.data[i].figure_name[1] = 0;
        ui_frame1_DynamicNumberGroup_0.data[i].figure_name[2] = i + 0;
        ui_frame1_DynamicNumberGroup_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_frame1_DynamicNumberGroup_0.data[i].operate_type = 0;
    }

    ui_frame1_DynamicNumberGroup_NewNumber->figure_type = 6;
    ui_frame1_DynamicNumberGroup_NewNumber->operate_type = 1;
    ui_frame1_DynamicNumberGroup_NewNumber->layer = 0;
    ui_frame1_DynamicNumberGroup_NewNumber->color = 2;
    ui_frame1_DynamicNumberGroup_NewNumber->start_x = 400;
    ui_frame1_DynamicNumberGroup_NewNumber->start_y = 813;
    ui_frame1_DynamicNumberGroup_NewNumber->width = 2;
    ui_frame1_DynamicNumberGroup_NewNumber->font_size = 20;
    ui_frame1_DynamicNumberGroup_NewNumber->number = 0;


    ui_proc_1_frame(&ui_frame1_DynamicNumberGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_DynamicNumberGroup_0, sizeof(ui_frame1_DynamicNumberGroup_0));
}

void _ui_update_frame1_DynamicNumberGroup_0() {
    for (int i = 0; i < 1; i++) {
        ui_frame1_DynamicNumberGroup_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_frame1_DynamicNumberGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_DynamicNumberGroup_0, sizeof(ui_frame1_DynamicNumberGroup_0));
}

void _ui_remove_frame1_DynamicNumberGroup_0() {
    for (int i = 0; i < 1; i++) {
        ui_frame1_DynamicNumberGroup_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_frame1_DynamicNumberGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_DynamicNumberGroup_0, sizeof(ui_frame1_DynamicNumberGroup_0));
}


void ui_init_frame1_DynamicNumberGroup() {
    _ui_init_frame1_DynamicNumberGroup_0();
}

void ui_update_frame1_DynamicNumberGroup() {
    _ui_update_frame1_DynamicNumberGroup_0();
}

void ui_remove_frame1_DynamicNumberGroup() {
    _ui_remove_frame1_DynamicNumberGroup_0();
}

ui_5_frame_t ui_frame1_StaticNumberGroup_0;

ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber1 = (ui_interface_number_t*)&(ui_frame1_StaticNumberGroup_0.data[0]);
ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber2 = (ui_interface_number_t*)&(ui_frame1_StaticNumberGroup_0.data[1]);
ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber3 = (ui_interface_number_t*)&(ui_frame1_StaticNumberGroup_0.data[2]);
ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber4 = (ui_interface_number_t*)&(ui_frame1_StaticNumberGroup_0.data[3]);
ui_interface_number_t *ui_frame1_StaticNumberGroup_NewNumber5 = (ui_interface_number_t*)&(ui_frame1_StaticNumberGroup_0.data[4]);

void _ui_init_frame1_StaticNumberGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_frame1_StaticNumberGroup_0.data[i].figure_name[0] = 1;
        ui_frame1_StaticNumberGroup_0.data[i].figure_name[1] = 1;
        ui_frame1_StaticNumberGroup_0.data[i].figure_name[2] = i + 0;
        ui_frame1_StaticNumberGroup_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_frame1_StaticNumberGroup_0.data[i].operate_type = 0;
    }

    ui_frame1_StaticNumberGroup_NewNumber1->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber1->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber1->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber1->color = 8;
    ui_frame1_StaticNumberGroup_NewNumber1->start_x = 1371;
    ui_frame1_StaticNumberGroup_NewNumber1->start_y = 569;
    ui_frame1_StaticNumberGroup_NewNumber1->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber1->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber1->number = 0;

    ui_frame1_StaticNumberGroup_NewNumber2->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber2->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber2->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber2->color = 7;
    ui_frame1_StaticNumberGroup_NewNumber2->start_x = 1356;
    ui_frame1_StaticNumberGroup_NewNumber2->start_y = 419;
    ui_frame1_StaticNumberGroup_NewNumber2->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber2->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber2->number = 20;

    ui_frame1_StaticNumberGroup_NewNumber3->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber3->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber3->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber3->color = 8;
    ui_frame1_StaticNumberGroup_NewNumber3->start_x = 1280;
    ui_frame1_StaticNumberGroup_NewNumber3->start_y = 826;
    ui_frame1_StaticNumberGroup_NewNumber3->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber3->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber3->number = 40;

    ui_frame1_StaticNumberGroup_NewNumber4->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber4->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber4->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber4->color = 7;
    ui_frame1_StaticNumberGroup_NewNumber4->start_x = 1304;
    ui_frame1_StaticNumberGroup_NewNumber4->start_y = 305;
    ui_frame1_StaticNumberGroup_NewNumber4->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber4->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber4->number = 40;

    ui_frame1_StaticNumberGroup_NewNumber5->figure_type = 6;
    ui_frame1_StaticNumberGroup_NewNumber5->operate_type = 1;
    ui_frame1_StaticNumberGroup_NewNumber5->layer = 0;
    ui_frame1_StaticNumberGroup_NewNumber5->color = 8;
    ui_frame1_StaticNumberGroup_NewNumber5->start_x = 1348;
    ui_frame1_StaticNumberGroup_NewNumber5->start_y = 721;
    ui_frame1_StaticNumberGroup_NewNumber5->width = 2;
    ui_frame1_StaticNumberGroup_NewNumber5->font_size = 15;
    ui_frame1_StaticNumberGroup_NewNumber5->number = 20;


    ui_proc_5_frame(&ui_frame1_StaticNumberGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticNumberGroup_0, sizeof(ui_frame1_StaticNumberGroup_0));
}

void _ui_update_frame1_StaticNumberGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_frame1_StaticNumberGroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_frame1_StaticNumberGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticNumberGroup_0, sizeof(ui_frame1_StaticNumberGroup_0));
}

void _ui_remove_frame1_StaticNumberGroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_frame1_StaticNumberGroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_frame1_StaticNumberGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticNumberGroup_0, sizeof(ui_frame1_StaticNumberGroup_0));
}


void ui_init_frame1_StaticNumberGroup() {
    _ui_init_frame1_StaticNumberGroup_0();
}

void ui_update_frame1_StaticNumberGroup() {
    _ui_update_frame1_StaticNumberGroup_0();
}

void ui_remove_frame1_StaticNumberGroup() {
    _ui_remove_frame1_StaticNumberGroup_0();
}


ui_string_frame_t ui_frame1_StaticTextGroup_0;
ui_interface_string_t* ui_frame1_StaticTextGroup_HeatText = &(ui_frame1_StaticTextGroup_0.option);

void _ui_init_frame1_StaticTextGroup_0() {
    ui_frame1_StaticTextGroup_0.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_0.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_0.option.figure_name[2] = 0;
    ui_frame1_StaticTextGroup_0.option.operate_type = 1;

    ui_frame1_StaticTextGroup_HeatText->figure_type = 7;
    ui_frame1_StaticTextGroup_HeatText->operate_type = 1;
    ui_frame1_StaticTextGroup_HeatText->layer = 0;
    ui_frame1_StaticTextGroup_HeatText->color = 5;
    ui_frame1_StaticTextGroup_HeatText->start_x = 560;
    ui_frame1_StaticTextGroup_HeatText->start_y = 700;
    ui_frame1_StaticTextGroup_HeatText->width = 2;
    ui_frame1_StaticTextGroup_HeatText->font_size = 20;
    ui_frame1_StaticTextGroup_HeatText->str_length = 1;
    strcpy(ui_frame1_StaticTextGroup_HeatText->string, "H");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_0, sizeof(ui_frame1_StaticTextGroup_0));
}

void _ui_update_frame1_StaticTextGroup_0() {
    ui_frame1_StaticTextGroup_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_0, sizeof(ui_frame1_StaticTextGroup_0));
}

void _ui_remove_frame1_StaticTextGroup_0() {
    ui_frame1_StaticTextGroup_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_0);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_0, sizeof(ui_frame1_StaticTextGroup_0));
}
ui_string_frame_t ui_frame1_StaticTextGroup_1;
ui_interface_string_t* ui_frame1_StaticTextGroup_PowerText = &(ui_frame1_StaticTextGroup_1.option);

void _ui_init_frame1_StaticTextGroup_1() {
    ui_frame1_StaticTextGroup_1.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_1.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_1.option.figure_name[2] = 1;
    ui_frame1_StaticTextGroup_1.option.operate_type = 1;

    ui_frame1_StaticTextGroup_PowerText->figure_type = 7;
    ui_frame1_StaticTextGroup_PowerText->operate_type = 1;
    ui_frame1_StaticTextGroup_PowerText->layer = 0;
    ui_frame1_StaticTextGroup_PowerText->color = 3;
    ui_frame1_StaticTextGroup_PowerText->start_x = 587;
    ui_frame1_StaticTextGroup_PowerText->start_y = 540;
    ui_frame1_StaticTextGroup_PowerText->width = 2;
    ui_frame1_StaticTextGroup_PowerText->font_size = 20;
    ui_frame1_StaticTextGroup_PowerText->str_length = 1;
    strcpy(ui_frame1_StaticTextGroup_PowerText->string, "P");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_1, sizeof(ui_frame1_StaticTextGroup_1));
}

void _ui_update_frame1_StaticTextGroup_1() {
    ui_frame1_StaticTextGroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_1, sizeof(ui_frame1_StaticTextGroup_1));
}

void _ui_remove_frame1_StaticTextGroup_1() {
    ui_frame1_StaticTextGroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_1);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_1, sizeof(ui_frame1_StaticTextGroup_1));
}
ui_string_frame_t ui_frame1_StaticTextGroup_2;
ui_interface_string_t* ui_frame1_StaticTextGroup_FricText = &(ui_frame1_StaticTextGroup_2.option);

void _ui_init_frame1_StaticTextGroup_2() {
    ui_frame1_StaticTextGroup_2.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_2.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_2.option.figure_name[2] = 2;
    ui_frame1_StaticTextGroup_2.option.operate_type = 1;

    ui_frame1_StaticTextGroup_FricText->figure_type = 7;
    ui_frame1_StaticTextGroup_FricText->operate_type = 1;
    ui_frame1_StaticTextGroup_FricText->layer = 0;
    ui_frame1_StaticTextGroup_FricText->color = 5;
    ui_frame1_StaticTextGroup_FricText->start_x = 880;
    ui_frame1_StaticTextGroup_FricText->start_y = 210;
    ui_frame1_StaticTextGroup_FricText->width = 2;
    ui_frame1_StaticTextGroup_FricText->font_size = 20;
    ui_frame1_StaticTextGroup_FricText->str_length = 1;
    strcpy(ui_frame1_StaticTextGroup_FricText->string, "F");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_2, sizeof(ui_frame1_StaticTextGroup_2));
}

void _ui_update_frame1_StaticTextGroup_2() {
    ui_frame1_StaticTextGroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_2, sizeof(ui_frame1_StaticTextGroup_2));
}

void _ui_remove_frame1_StaticTextGroup_2() {
    ui_frame1_StaticTextGroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_2);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_2, sizeof(ui_frame1_StaticTextGroup_2));
}
ui_string_frame_t ui_frame1_StaticTextGroup_3;
ui_interface_string_t* ui_frame1_StaticTextGroup_AutoText = &(ui_frame1_StaticTextGroup_3.option);

void _ui_init_frame1_StaticTextGroup_3() {
    ui_frame1_StaticTextGroup_3.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_3.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_3.option.figure_name[2] = 3;
    ui_frame1_StaticTextGroup_3.option.operate_type = 1;

    ui_frame1_StaticTextGroup_AutoText->figure_type = 7;
    ui_frame1_StaticTextGroup_AutoText->operate_type = 1;
    ui_frame1_StaticTextGroup_AutoText->layer = 0;
    ui_frame1_StaticTextGroup_AutoText->color = 5;
    ui_frame1_StaticTextGroup_AutoText->start_x = 960;
    ui_frame1_StaticTextGroup_AutoText->start_y = 210;
    ui_frame1_StaticTextGroup_AutoText->width = 2;
    ui_frame1_StaticTextGroup_AutoText->font_size = 20;
    ui_frame1_StaticTextGroup_AutoText->str_length = 1;
    strcpy(ui_frame1_StaticTextGroup_AutoText->string, "A");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_3);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_3, sizeof(ui_frame1_StaticTextGroup_3));
}

void _ui_update_frame1_StaticTextGroup_3() {
    ui_frame1_StaticTextGroup_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_3);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_3, sizeof(ui_frame1_StaticTextGroup_3));
}

void _ui_remove_frame1_StaticTextGroup_3() {
    ui_frame1_StaticTextGroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_3);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_3, sizeof(ui_frame1_StaticTextGroup_3));
}
ui_string_frame_t ui_frame1_StaticTextGroup_4;
ui_interface_string_t* ui_frame1_StaticTextGroup_Text1 = &(ui_frame1_StaticTextGroup_4.option);

void _ui_init_frame1_StaticTextGroup_4() {
    ui_frame1_StaticTextGroup_4.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_4.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_4.option.figure_name[2] = 4;
    ui_frame1_StaticTextGroup_4.option.operate_type = 1;

    ui_frame1_StaticTextGroup_Text1->figure_type = 7;
    ui_frame1_StaticTextGroup_Text1->operate_type = 1;
    ui_frame1_StaticTextGroup_Text1->layer = 0;
    ui_frame1_StaticTextGroup_Text1->color = 3;
    ui_frame1_StaticTextGroup_Text1->start_x = 113;
    ui_frame1_StaticTextGroup_Text1->start_y = 813;
    ui_frame1_StaticTextGroup_Text1->width = 2;
    ui_frame1_StaticTextGroup_Text1->font_size = 20;
    ui_frame1_StaticTextGroup_Text1->str_length = 12;
    strcpy(ui_frame1_StaticTextGroup_Text1->string, "bullet_speed");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_4);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_4, sizeof(ui_frame1_StaticTextGroup_4));
}

void _ui_update_frame1_StaticTextGroup_4() {
    ui_frame1_StaticTextGroup_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_4);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_4, sizeof(ui_frame1_StaticTextGroup_4));
}

void _ui_remove_frame1_StaticTextGroup_4() {
    ui_frame1_StaticTextGroup_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_4);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_4, sizeof(ui_frame1_StaticTextGroup_4));
}
ui_string_frame_t ui_frame1_StaticTextGroup_5;
ui_interface_string_t* ui_frame1_StaticTextGroup_Text2 = &(ui_frame1_StaticTextGroup_5.option);

void _ui_init_frame1_StaticTextGroup_5() {
    ui_frame1_StaticTextGroup_5.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_5.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_5.option.figure_name[2] = 5;
    ui_frame1_StaticTextGroup_5.option.operate_type = 1;

    ui_frame1_StaticTextGroup_Text2->figure_type = 7;
    ui_frame1_StaticTextGroup_Text2->operate_type = 1;
    ui_frame1_StaticTextGroup_Text2->layer = 0;
    ui_frame1_StaticTextGroup_Text2->color = 2;
    ui_frame1_StaticTextGroup_Text2->start_x = 113;
    ui_frame1_StaticTextGroup_Text2->start_y = 390;
    ui_frame1_StaticTextGroup_Text2->width = 2;
    ui_frame1_StaticTextGroup_Text2->font_size = 20;
    ui_frame1_StaticTextGroup_Text2->str_length = 5;
    strcpy(ui_frame1_StaticTextGroup_Text2->string, "hight");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_5);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_5, sizeof(ui_frame1_StaticTextGroup_5));
}

void _ui_update_frame1_StaticTextGroup_5() {
    ui_frame1_StaticTextGroup_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_5);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_5, sizeof(ui_frame1_StaticTextGroup_5));
}

void _ui_remove_frame1_StaticTextGroup_5() {
    ui_frame1_StaticTextGroup_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_5);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_5, sizeof(ui_frame1_StaticTextGroup_5));
}

ui_string_frame_t ui_frame1_StaticTextGroup_6;
ui_interface_string_t* ui_frame1_StaticTextGroup_Text3 = &(ui_frame1_StaticTextGroup_6.option);

void _ui_init_frame1_StaticTextGroup_6() {
    ui_frame1_StaticTextGroup_6.option.figure_name[0] = 1;
    ui_frame1_StaticTextGroup_6.option.figure_name[1] = 2;
    ui_frame1_StaticTextGroup_6.option.figure_name[2] = 6;
    ui_frame1_StaticTextGroup_6.option.operate_type = 1;

    ui_frame1_StaticTextGroup_Text3->figure_type = 7;
    ui_frame1_StaticTextGroup_Text3->operate_type = 1;
    ui_frame1_StaticTextGroup_Text3->layer = 0;
    ui_frame1_StaticTextGroup_Text3->color = 2;
    ui_frame1_StaticTextGroup_Text3->start_x = 113;
    ui_frame1_StaticTextGroup_Text3->start_y = 720;
    ui_frame1_StaticTextGroup_Text3->width = 2;
    ui_frame1_StaticTextGroup_Text3->font_size = 20;
    ui_frame1_StaticTextGroup_Text3->str_length = 9;
    strcpy(ui_frame1_StaticTextGroup_Text3->string, "max_hight");


    ui_proc_string_frame(&ui_frame1_StaticTextGroup_6);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_6, sizeof(ui_frame1_StaticTextGroup_6));
}

void _ui_update_frame1_StaticTextGroup_6() {
    ui_frame1_StaticTextGroup_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_6);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_6, sizeof(ui_frame1_StaticTextGroup_6));
}

void _ui_remove_frame1_StaticTextGroup_6() {
    ui_frame1_StaticTextGroup_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_frame1_StaticTextGroup_6);
    SEND_MESSAGE((uint8_t *) &ui_frame1_StaticTextGroup_6, sizeof(ui_frame1_StaticTextGroup_6));
}
void ui_init_frame1_StaticTextGroup() {
    _ui_init_frame1_StaticTextGroup_0();
    _ui_init_frame1_StaticTextGroup_1();
    _ui_init_frame1_StaticTextGroup_2();
    _ui_init_frame1_StaticTextGroup_3();
    _ui_init_frame1_StaticTextGroup_4();
    _ui_init_frame1_StaticTextGroup_5();
    _ui_init_frame1_StaticTextGroup_6();
}

void ui_update_frame1_StaticTextGroup() {
    _ui_update_frame1_StaticTextGroup_0();
    _ui_update_frame1_StaticTextGroup_1();
    _ui_update_frame1_StaticTextGroup_2();
    _ui_update_frame1_StaticTextGroup_3();
    _ui_update_frame1_StaticTextGroup_4();
    _ui_update_frame1_StaticTextGroup_5();
    _ui_update_frame1_StaticTextGroup_6();

}

void ui_remove_frame1_StaticTextGroup() {
    _ui_remove_frame1_StaticTextGroup_0();
    _ui_remove_frame1_StaticTextGroup_1();
    _ui_remove_frame1_StaticTextGroup_2();
    _ui_remove_frame1_StaticTextGroup_3();
    _ui_remove_frame1_StaticTextGroup_4();
    _ui_remove_frame1_StaticTextGroup_5();
    _ui_remove_frame1_StaticTextGroup_6();
}

