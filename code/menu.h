#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"

#define BUTTON_LF (P19_4)
#define BUTTON_RT (P20_3)
#define BUTTON_DN (P20_0)
#define BUTTON_UP (P20_2)
#define BUTTON_OK (P20_1)

#define DIP_1 (P08_3)
#define DIP_2 (P09_0)
#define DIP_3 (P09_1)
#define DIP_4 (P10_2)

#define MAX_DISPLAY 10
#define MAX_LAYER 3

#define MENU1_FLASH_PAGE 0

typedef enum
{
    MENU_TYPE_NORMAL,
    MENU_TYPE_INT,
    MENU_TYPE_FLOAT,
    MENU_TYPE_FUNCTION
} MENU_Type;

typedef struct Menu
{
    char Name[20];
    uint8_t layer;
    uint16_t order;
    MENU_Type type;
    int data_int;
    float data_float;
    void (*Function)(void);
    uint8_t exec_flag;

    struct Menu *last;
    struct Menu *next;
    struct Menu *parent;
    struct Menu *child;
} MENU;

typedef enum
{
    MENU_STATE_CONTROL,
    MENU_STATE_INS,
} menu_state_t;

extern MENU *RootMenu, *Balance, *Image, *Element, *currentMenu;
extern MENU *speed, *anloop1, *anloop2, *sploop, *tuloop1, *tuloop2, *asloop, *snloop, *yaloop, *pzero, *pinit, *show,
    *thre, *farl, *nearl, *saveim,
    *change, *snspeed, *rollba, *turnamf, *turnamt, *testnum, *cwtucop, *ccwtucop, *saveel;
extern MENU *an1kp, *an1ki, *an1kd,
    *an2kp, *an2ki, *an2kd,
    *spkp, *spki, *spkd,
    *tu1kp, *tu1ki, *tu1kd,
    *tu2kp, *tu2ki, *tu2kd,
    *askp, *aski, *askd,
    *snkp, *snki, *snkd,
    *yakp, *yaki, *yakd,
    *zeroan1, *zeroan2, *zerosp, *zerotu1, *zerotu2, *zeroas, *zerosn, *zeroya,
    *savean1, *savean2, *savesp, *savetu1, *savetu2, *saveas, *savesn, *saveya;

extern volatile menu_state_t menu_current_state;

MENU *createMenu(const char *name, uint8_t layer, uint16_t order, MENU_Type type, void (*func)(void));
uint16_t countNodes(MENU *layerHead);
void clear_exec_flag(MENU *layerHead);
void create_1st_menu(void);
void create_2nd_menu(void);
void create_3rd_menu(void);
void show_menu(void);
void key_handle_menu(void);

void menu_save(void);
void menu_load1(void);
void menu_zero(void);
static void image_show(void);
void ins_to_control_menu(void);
void control_to_ins_menu(void);
void key_handle_ins(void);

#endif /* CODE_MENU_H_ */
