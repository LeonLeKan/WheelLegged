#include "menu.h"
#include "image.h"
#include "imu.h"
#include "screen_interface.h"
#include "save_manager.h"

volatile menu_state_t menu_current_state = MENU_STATE_CONTROL;

MENU *RootMenu, *Balance, *Image, *Element, *currentMenu;
MENU *speed, *anloop1, *anloop2, *sploop, *tuloop1, *tuloop2, *asloop, *snloop, *pzero, *pinit, *show,
    *thre, *farl, *nearl, *saveim,
    *change;
MENU *an1kp, *an1ki, *an1kd,
    *an2kp, *an2ki, *an2kd,
    *spkp, *spki, *spkd,
    *tu1kp, *tu1ki, *tu1kd,
    *tu2kp, *tu2ki, *tu2kd,
    *askp, *aski, *askd,
    *snkp, *snki, *snkd,
    *zeroan1, *zeroan2, *zerosp, *zerotu1, *zerotu2, *zeroas, *zerosn,
    *savean1, *savean2, *savesp, *savetu1, *savetu2, *saveas, *savesn;

#define MENU_POOL_SIZE 128

static MENU menu_pool[MENU_POOL_SIZE];
static uint16_t menu_pool_used = 0;

uint8_t edit_mode = 0; // 0=正常菜单模式, 1=编辑模式
uint8_t edit_pos = 0;  // 当前编辑位（0=符号，float的小数点也占一位）
char edit_buf[12];     // 存储临时的数字字符数组

float menu_magic = 1.14514f;

MENU *createMenu(const char *name, uint8_t layer, uint16_t order, MENU_Type type, void (*func)(void))
{
    MENU *newNode = &menu_pool[menu_pool_used++];
    memset(newNode, 0, sizeof(MENU));

    strncpy(newNode->Name, name, sizeof(newNode->Name) - 1);
    newNode->Name[sizeof(newNode->Name) - 1] = '\0';

    newNode->layer = layer;
    newNode->order = order;
    newNode->type = type;
    newNode->Function = func;

    newNode->data_int = 0;
    newNode->data_float = 0.0f;
    newNode->exec_flag = 0;

    newNode->last = NULL;
    newNode->next = NULL;
    newNode->parent = NULL;
    newNode->child = NULL;

    return newNode;
}

uint16_t countNodes(MENU *layerHead)
{
    uint16_t count = 1;
    MENU *temp = layerHead->next;
    while (temp != layerHead)
    {
        count++;
        temp = temp->next;
    }
    return count;
}

void clear_exec_flag(MENU *layerHead)
{
    if (!layerHead)
    {
        return;
    }
    MENU *current = layerHead;
    do
    {
        current->exec_flag = 0;
        current = current->next;
    } while (current != layerHead);
}

void create_1st_menu(void)
{
    RootMenu = createMenu("RootMenu", 0, 1, MENU_TYPE_NORMAL, NULL);
    Balance = createMenu("Balance", 1, 1, MENU_TYPE_NORMAL, NULL);
    Image = createMenu("Image", 1, 2, MENU_TYPE_NORMAL, NULL);
    Element = createMenu("Element", 1, 3, MENU_TYPE_NORMAL, NULL);

    RootMenu->child = Balance;
    Balance->parent = RootMenu;
    Image->parent = RootMenu;
    Element->parent = RootMenu;

    Balance->next = Image;
    Image->last = Balance;
    Image->next = Element;
    Element->last = Image;
    Element->next = Balance;
    Balance->last = Element;

    currentMenu = RootMenu->child;
}

void create_2nd_menu(void)
{
    speed = createMenu("speed", 2, 1, MENU_TYPE_FLOAT, NULL);
    anloop1 = createMenu("anloop1", 2, 2, MENU_TYPE_NORMAL, NULL);
    anloop2 = createMenu("anloop2", 2, 3, MENU_TYPE_NORMAL, NULL);
    sploop = createMenu("sploop", 2, 4, MENU_TYPE_NORMAL, NULL);
    tuloop1 = createMenu("tuloop1", 2, 5, MENU_TYPE_NORMAL, NULL);
    tuloop2 = createMenu("tuloop2", 2, 6, MENU_TYPE_NORMAL, NULL);
    asloop = createMenu("asloop", 2, 7, MENU_TYPE_NORMAL, NULL);
    snloop = createMenu("snloop", 2, 8, MENU_TYPE_NORMAL, NULL);
    pzero = createMenu("pzero", 2, 9, MENU_TYPE_FLOAT, NULL);
    pinit = createMenu("pinit", 2, 10, MENU_TYPE_FLOAT, NULL);
    show = createMenu("show", 2, 11, MENU_TYPE_FUNCTION, image_show);

    thre = createMenu("thre", 2, 1, MENU_TYPE_INT, NULL);
    farl = createMenu("farl", 2, 2, MENU_TYPE_INT, NULL);
    nearl = createMenu("nearl", 2, 3, MENU_TYPE_INT, NULL);
    saveim = createMenu("save", 2, 4, MENU_TYPE_FUNCTION, menu_save1);

    change = createMenu("change", 2, 1, MENU_TYPE_FUNCTION, control_to_ins_menu);

    speed->next = anloop1;
    anloop1->last = speed;
    anloop1->next = anloop2;
    anloop2->last = anloop1;
    anloop2->next = sploop;
    sploop->last = anloop2;
    sploop->next = tuloop1;
    tuloop1->last = sploop;
    tuloop1->next = tuloop2;
    tuloop2->last = tuloop1;
    tuloop2->next = asloop;
    asloop->last = tuloop2;
    asloop->next = snloop;
    snloop->last = asloop;
    snloop->next = pzero;
    pzero->last = snloop;
    pzero->next = pinit;
    pinit->last = pzero;
    pinit->next = show;
    show->last = pinit;
    show->next = speed;
    speed->last = show;

    thre->next = farl;
    farl->last = thre;
    farl->next = nearl;
    nearl->last = farl;
    nearl->next = saveim;
    saveim->last = nearl;
    saveim->next = thre;
    thre->last = saveim;

    change->next = change;
    change->last = change;

    Balance->child = speed;
    speed->parent = Balance;
    anloop1->parent = Balance;
    anloop2->parent = Balance;
    sploop->parent = Balance;
    tuloop1->parent = Balance;
    tuloop2->parent = Balance;
    asloop->parent = Balance;
    snloop->parent = Balance;
    pzero->parent = Balance;
    pinit->parent = Balance;
    show->parent = Balance;

    Image->child = thre;
    thre->parent = Image;
    farl->parent = Image;
    nearl->parent = Image;
    saveim->parent = Image;

    Element->child = change;
    change->parent = Element;
}

void create_3rd_menu(void)
{
    an1kp = createMenu("an1kp", 3, 1, MENU_TYPE_FLOAT, NULL);
    an1ki = createMenu("an1ki", 3, 2, MENU_TYPE_FLOAT, NULL);
    an1kd = createMenu("an1kd", 3, 3, MENU_TYPE_FLOAT, NULL);

    an2kp = createMenu("an2kp", 3, 1, MENU_TYPE_FLOAT, NULL);
    an2ki = createMenu("an2ki", 3, 2, MENU_TYPE_FLOAT, NULL);
    an2kd = createMenu("an2kd", 3, 3, MENU_TYPE_FLOAT, NULL);

    spkp = createMenu("spkp", 3, 1, MENU_TYPE_FLOAT, NULL);
    spki = createMenu("spki", 3, 2, MENU_TYPE_FLOAT, NULL);
    spkd = createMenu("spkd", 3, 3, MENU_TYPE_FLOAT, NULL);

    tu1kp = createMenu("tu1kp", 3, 1, MENU_TYPE_FLOAT, NULL);
    tu1ki = createMenu("tu1ki", 3, 2, MENU_TYPE_FLOAT, NULL);
    tu1kd = createMenu("tu1kd", 3, 3, MENU_TYPE_FLOAT, NULL);

    tu2kp = createMenu("tu2kp", 3, 1, MENU_TYPE_FLOAT, NULL);
    tu2ki = createMenu("tu2ki", 3, 2, MENU_TYPE_FLOAT, NULL);
    tu2kd = createMenu("tu2kd", 3, 3, MENU_TYPE_FLOAT, NULL);

    askp = createMenu("askp", 3, 1, MENU_TYPE_FLOAT, NULL);
    aski = createMenu("aski", 3, 2, MENU_TYPE_FLOAT, NULL);
    askd = createMenu("askd", 3, 3, MENU_TYPE_FLOAT, NULL);

    snkp = createMenu("snkp", 3, 1, MENU_TYPE_FLOAT, NULL);
    snki = createMenu("snki", 3, 2, MENU_TYPE_FLOAT, NULL);
    snkd = createMenu("snkd", 3, 3, MENU_TYPE_FLOAT, NULL);

    savean1 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);
    savean2 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);
    savesp = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);
    savetu1 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);
    savetu2 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);
    saveas = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);
    savesn = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save1);

    zeroan1 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zeroan2 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerosp = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerotu1 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerotu2 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zeroas = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerosn = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);

    anloop1->child = an1kp;
    an1kp->next = an1ki;
    an1ki->last = an1kp;
    an1ki->next = an1kd;
    an1kd->last = an1ki;
    an1kd->next = savean1;
    savean1->last = an1kd;
    savean1->next = zeroan1;
    zeroan1->last = savean1;
    zeroan1->next = an1kp;
    an1kp->last = zeroan1;

    anloop2->child = an2kp;
    an2kp->next = an2ki;
    an2ki->last = an2kp;
    an2ki->next = an2kd;
    an2kd->last = an2ki;
    an2kd->next = savean2;
    savean2->last = an2kd;
    savean2->next = zeroan2;
    zeroan2->last = savean2;
    zeroan2->next = an2kp;
    an2kp->last = zeroan2;

    sploop->child = spkp;
    spkp->next = spki;
    spki->last = spkp;
    spki->next = spkd;
    spkd->last = spki;
    spkd->next = savesp;
    savesp->last = spkd;
    savesp->next = zerosp;
    zerosp->last = savesp;
    zerosp->next = spkp;
    spkp->last = zerosp;

    tuloop1->child = tu1kp;
    tu1kp->next = tu1ki;
    tu1ki->last = tu1kp;
    tu1ki->next = tu1kd;
    tu1kd->last = tu1ki;
    tu1kd->next = savetu1;
    savetu1->last = tu1kd;
    savetu1->next = zerotu1;
    zerotu1->last = savetu1;
    zerotu1->next = tu1kp;
    tu1kp->last = zerotu1;

    tuloop2->child = tu2kp;
    tu2kp->next = tu2ki;
    tu2ki->last = tu2kp;
    tu2ki->next = tu2kd;
    tu2kd->last = tu2ki;
    tu2kd->next = savetu2;
    savetu2->last = tu2kd;
    savetu2->next = zerotu2;
    zerotu2->last = savetu2;
    zerotu2->next = tu2kp;
    tu2kp->last = zerotu2;

    asloop->child = askp;
    askp->next = aski;
    aski->last = askp;
    aski->next = askd;
    askd->last = aski;
    askd->next = saveas;
    saveas->last = askd;
    saveas->next = zeroas;
    zeroas->last = saveas;
    zeroas->next = askp;
    askp->last = zeroas;

    snloop->child = snkp;
    snkp->next = snki;
    snki->last = snkp;
    snki->next = snkd;
    snkd->last = snki;
    snkd->next = savesn;
    savesn->last = snkd;
    savesn->next = zerosn;
    zerosn->last = savesn;
    zerosn->next = snkp;
    snkp->last = zerosn;

    an1kp->parent = anloop1;
    an1ki->parent = anloop1;
    an1kd->parent = anloop1;
    zeroan1->parent = anloop1;
    savean1->parent = anloop1;

    an2kp->parent = anloop2;
    an2ki->parent = anloop2;
    an2kd->parent = anloop2;
    zeroan2->parent = anloop2;
    savean2->parent = anloop2;

    spkp->parent = sploop;
    spki->parent = sploop;
    spkd->parent = sploop;
    zerosp->parent = sploop;
    savesp->parent = sploop;

    tu1kp->parent = tuloop1;
    tu1ki->parent = tuloop1;
    tu1kd->parent = tuloop1;
    zerotu1->parent = tuloop1;
    savetu1->parent = tuloop1;

    tu2kp->parent = tuloop2;
    tu2ki->parent = tuloop2;
    tu2kd->parent = tuloop2;
    zerotu2->parent = tuloop2;
    savetu2->parent = tuloop2;

    askp->parent = asloop;
    aski->parent = asloop;
    askd->parent = asloop;
    zeroas->parent = asloop;
    saveas->parent = asloop;

    snkp->parent = snloop;
    snki->parent = snloop;
    snkd->parent = snloop;
    zerosn->parent = snloop;
    savesn->parent = snloop;
}

void key_handle_menu(void)
{
    static uint8_t last_switch_state = 1; // 上电前拉回高电平
    uint8_t current_switch_state = gpio_get_level(BUTTON_OK);

    uint8_t switch_changed = (last_switch_state != current_switch_state);

    if (switch_changed)
    {
        if (!edit_mode)
        {
            if (currentMenu->type == MENU_TYPE_FUNCTION && currentMenu->Function != NULL)
            {
                currentMenu->Function();
                currentMenu->exec_flag = 1;
                ips200_clear();
            }
        }
        last_switch_state = current_switch_state;
        return;
    }

    if (!edit_mode)
    {
        if (gpio_get_level(BUTTON_DN) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_DN) == 0)
            {
                clear_exec_flag(currentMenu->parent->child);
                currentMenu = currentMenu->next;
            }
        }
        else if (gpio_get_level(BUTTON_UP) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_UP) == 0)
            {
                clear_exec_flag(currentMenu->parent->child);
                currentMenu = currentMenu->last;
            }
        }
        else if (gpio_get_level(BUTTON_LF) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_LF) == 0)
            {
                if (currentMenu->parent != NULL && currentMenu->layer != 1)
                {
                    clear_exec_flag(currentMenu->parent->child);
                    currentMenu = currentMenu->parent;
                }
            }
        }
        else if (gpio_get_level(BUTTON_RT) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_RT) == 0)
            {
                if (currentMenu->type == MENU_TYPE_INT || currentMenu->type == MENU_TYPE_FLOAT)
                {
                    edit_mode = 1;
                    edit_pos = 0;
                    if (currentMenu->type == MENU_TYPE_INT)
                    {
                        sprintf(edit_buf, "%+09d", currentMenu->data_int);
                    }
                    else
                    {
                        sprintf(edit_buf, "%+09.3f", currentMenu->data_float);
                    }
                }
                else if (currentMenu->child != NULL && currentMenu->layer < MAX_LAYER)
                {
                    ips200_clear();
                    clear_exec_flag(currentMenu->child);
                    currentMenu = currentMenu->child;
                    printf("  currentMenu: %s (layer=%d)\n", currentMenu->Name, currentMenu->layer);
                }
            }
        }
    }
    else
    {
        if (gpio_get_level(BUTTON_RT) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_RT) == 0)
            {
                edit_pos++;
                if (edit_pos >= strlen(edit_buf))
                {
                    edit_pos = strlen(edit_buf) - 1;
                }
            }
        }
        else if (gpio_get_level(BUTTON_LF) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_LF) == 0)
            {
                if (edit_pos == 0)
                {
                    edit_mode = 0;
                }
                else
                {
                    edit_pos--;
                }
            }
        }
        else if (gpio_get_level(BUTTON_UP) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_UP) == 0)
            {
                char *ch = &edit_buf[edit_pos];

                if (*ch == '+' || *ch == '-')
                {
                    *ch = '+';
                }
                else if (*ch >= '0' && *ch <= '9')
                {
                    (*ch)++;
                    if (*ch > '9')
                    {
                        *ch = '0';
                    }
                }
            }
        }
        else if (gpio_get_level(BUTTON_DN) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_DN) == 0)
            {
                char *ch = &edit_buf[edit_pos];

                if (*ch == '+' || *ch == '-')
                {
                    *ch = '-';
                }
                else if (*ch >= '0' && *ch <= '9')
                {
                    (*ch)--;
                    if (*ch < '0')
                    {
                        *ch = '9';
                    }
                }
            }
        }
        if (currentMenu->type == MENU_TYPE_INT)
        {
            currentMenu->data_int = atoi(edit_buf);
        }
        else if (currentMenu->type == MENU_TYPE_FLOAT)
        {
            currentMenu->data_float = atof(edit_buf);
        }
    }
}

void show_menu(void)
{
    char buf[20];
    uint8_t highlight = 0;
    uint16_t count_node = countNodes(currentMenu->parent->child);
    sprintf(buf, "%s", currentMenu->parent->Name);
    ips200_show_string(0, 0, buf);
    sprintf(buf, "%02d/%02d", currentMenu->order, count_node);
    ips200_show_string(200, 0, buf);

    if (currentMenu->type == MENU_TYPE_INT || currentMenu->type == MENU_TYPE_FLOAT)
    {
        char *pstr;
        if (edit_mode)
        {
            pstr = edit_buf;
        }
        else
        {
            if (currentMenu->type == MENU_TYPE_INT)
            {
                sprintf(buf, "%+09d", currentMenu->data_int);
            }
            else
            {
                sprintf(buf, "%+09.3f", currentMenu->data_float);
            }
            pstr = buf;
        }

        for (int i = 0; i < strlen(pstr); i++)
        {
            printf("%s", pstr);
            highlight = (edit_mode && i == edit_pos) ? 0 : 1;
            ips200_show_char(168 + i * 8, 240, pstr[i]);
            if (highlight == 0)
            {
                ips200_show_string(168 + i * 8, 258, "^");
            }
        }
    }
    else
    {
        ips200_show_string(168, 240, "Wait Data");
    }
    if (currentMenu->exec_flag)
    {
        ips200_show_string(168, 16, "EXE:YES");
    }
    else
    {
        ips200_show_string(168, 16, "EXE:NO");
    }
    if (count_node <= MAX_DISPLAY)
    {
        MENU *temp = currentMenu->parent->child;
        uint32_t index = 16;
        do
        {
            highlight = (temp == currentMenu) ? 0 : 1;
            ips200_show_string(16, index, temp->Name);
            if (highlight == 0)
            {
                ips200_show_string(4, index, ">");
            }
            temp = temp->next;
            index += 16;
        } while (temp != currentMenu->parent->child);
    }
    else
    {
        static MENU *window_start = NULL;
        if (window_start == NULL || window_start->parent != currentMenu->parent)
        {
            window_start = currentMenu->parent->child;
        }
        MENU *temp = window_start;
        int found = 0;
        for (int i = 0; i < MAX_DISPLAY; i++)
        {
            if (temp == currentMenu)
            {
                found = 1;
                break;
            }
            temp = temp->next;
        }
        if (!found)
        {
            if (currentMenu->order > window_start->order)
            {
                int target_pos = MAX_DISPLAY - 1;
                window_start = currentMenu;
                for (int i = 0; i < target_pos; i++)
                {
                    window_start = window_start->last;
                }
            }
            else
            {
                window_start = currentMenu;
            }
        }
        temp = window_start;
        uint32_t index = 16;
        for (int i = 0; i < MAX_DISPLAY; i++)
        {
            highlight = (temp == currentMenu) ? 0 : 1;
            ips200_show_string(16, index, temp->Name);
            if (highlight == 0)
            {
                ips200_show_string(4, index, ">");
            }
            temp = temp->next;
            index += 16;
        }
    }
    char buf_float_kp[20], buf_float_ki[20], buf_float_kd[20];

    MENU *kp_node = NULL, *ki_node = NULL, *kd_node = NULL;

    if (currentMenu->layer == 3)
    {
        MENU *second_menu = currentMenu->parent;
        if (second_menu && second_menu->child)
        {
            kp_node = second_menu->child;
            ki_node = kp_node->next;
            kd_node = ki_node->next;
        }
    }

    if (kp_node && ki_node && kd_node)
    {
        sprintf(buf_float_kp, "%+09.3f", kp_node->data_float);
        sprintf(buf_float_ki, "%+09.3f", ki_node->data_float);
        sprintf(buf_float_kd, "%+09.3f", kd_node->data_float);
    }
    else
    {
        sprintf(buf_float_kp, "%+09.3f", 0.0);
        sprintf(buf_float_ki, "%+09.3f", 0.0);
        sprintf(buf_float_kd, "%+09.3f", 0.0);
    }

    ips200_show_string(168, 192, buf_float_kp);
    ips200_show_string(168, 208, buf_float_ki);
    ips200_show_string(168, 224, buf_float_kd);
}

void menu_save1(void)
{
    flash_erase_page(0, MENU1_FLASH_PAGE);

    flash_buffer_clear();

    uint32_t index = 2; // 0: magic, 1: node_count
    uint32_t node_count = 0;

    MENU *loop = Balance->child;
    if (!loop)
        return;

    do
    {
        if (loop->type == MENU_TYPE_INT)
        {
            flash_union_buffer[index++].int32_type = loop->data_int;
            node_count++;
        }
        else if (loop->type == MENU_TYPE_FLOAT)
        {
            union
            {
                float f;
                uint32_t u;
            } fu;
            fu.f = loop->data_float;
            flash_union_buffer[index++].uint32_type = fu.u;
            node_count++;
        }

        MENU *param = loop->child;
        if (param)
        {
            MENU *tmp = param;
            do
            {
                if (tmp->type == MENU_TYPE_INT)
                {
                    flash_union_buffer[index++].int32_type = tmp->data_int;
                    node_count++;
                }
                else if (tmp->type == MENU_TYPE_FLOAT)
                {
                    union
                    {
                        float f;
                        uint32_t u;
                    } fu;
                    fu.f = tmp->data_float;
                    flash_union_buffer[index++].uint32_type = fu.u;
                    node_count++;
                }
                tmp = tmp->next;
            } while (tmp != param);
        }

        loop = loop->next;
    } while (loop != Balance->child);

    MENU *img = Image->child;
    if (img != NULL)
    {
        MENU *tmp = img;
        do
        {
            if (tmp->type == MENU_TYPE_INT)
            {
                flash_union_buffer[index++].int32_type = tmp->data_int;
                node_count++;
            }

            tmp = tmp->next;
        } while (tmp != img);
    }

    flash_union_buffer[0].float_type = menu_magic;
    flash_union_buffer[1].uint32_type = node_count;

    flash_erase_page(0, MENU1_FLASH_PAGE);
    flash_write_page_from_buffer(0, MENU1_FLASH_PAGE, index);
}

void menu_load1(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, MENU1_FLASH_PAGE, FLASH_PAGE_LENGTH);

    if (flash_union_buffer[0].float_type != menu_magic)
        return;

    uint32_t idx = 2;
    uint32_t real_count = 0;

    MENU *loop = Balance->child;
    if (loop == NULL)
        return;

    do
    {
        if (loop->type == MENU_TYPE_INT)
        {
            loop->data_int = (int)flash_union_buffer[idx++].int32_type;
            if (fabs((float)loop->data_int) >= 100000)
            {
                loop->data_int = 0;
            }
            real_count++;
        }
        else if (loop->type == MENU_TYPE_FLOAT)
        {
            union
            {
                float f;
                uint32_t u;
            } fu;
            fu.u = flash_union_buffer[idx++].uint32_type;
            loop->data_float = fu.f;
            if (fabs(loop->data_float) >= 100000)
            {
                loop->data_float = 0.0f;
            }
            real_count++;
        }

        MENU *param = loop->child;
        if (param != NULL)
        {
            MENU *tmp = param;
            do
            {
                if (tmp->type == MENU_TYPE_INT)
                {
                    tmp->data_int = (int)flash_union_buffer[idx++].int32_type;
                    if (fabs((float)tmp->data_int) >= 100000)
                    {
                        tmp->data_int = 0;
                    }
                    real_count++;
                }
                else if (tmp->type == MENU_TYPE_FLOAT)
                {
                    union
                    {
                        float f;
                        uint32_t u;
                    } fu;
                    fu.u = flash_union_buffer[idx++].uint32_type;
                    tmp->data_float = fu.f;
                    if (fabs(tmp->data_float) >= 100000)
                    {
                        tmp->data_float = 0;
                    }
                    real_count++;
                }
                tmp = tmp->next;
            } while (tmp != param);
        }

        loop = loop->next;
    } while (loop != Balance->child);

    MENU *img = Image->child;
    if (img != NULL)
    {
        MENU *tmp = img;
        do
        {
            if (tmp->type == MENU_TYPE_INT)
            {
                tmp->data_int = (int)flash_union_buffer[idx++].int32_type;
                if (fabs((float)tmp->data_int) >= 100000)
                {
                    tmp->data_int = 0;
                }
                real_count++;
            }
            tmp = tmp->next;
        } while (tmp != img);
    }
}

void menu_zero(void)
{
    if (currentMenu == NULL || currentMenu->parent == NULL)
        return;

    MENU *loop = currentMenu->parent;
    MENU *param = loop->child;
    if (param == NULL)
        return;

    MENU *tmp = param;
    do
    {
        if (tmp->type == MENU_TYPE_INT)
        {
            tmp->data_int = 0;
        }
        else if (tmp->type == MENU_TYPE_FLOAT)
        {
            tmp->data_float = 0.0f;
        }
        tmp = tmp->next;
    } while (tmp != param);

    currentMenu->exec_flag = 1;
    ips200_clear();
}
uint8_t my_img[CAMERA_H][CAMERA_W];

static void image_show(void)
{
    ips200_clear();

    while (1)
    {
        image_main();
        for (int i = 0; i < CAMERA_H; i++)
        {
            for (int j = 0; j < CAMERA_W; j++)
            {
                if (IMG[i][j])
                    my_img[i][j] = 255;
                else
                    my_img[i][j] = 0;
            }
        }
        // ips200_displayimage03x((const uint8 *)mt9v03x_image, 240, 150);
        ips200_show_gray_image(0, 0, &my_img[0][0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 200);
        if (gpio_get_level(BUTTON_LF) == 0)
        {
            system_delay_ms(80);
            if (gpio_get_level(BUTTON_LF) == 0)
            {
                break;
            }
        }
        system_delay_ms(30);
    }
    ips200_clear();
}

void ins_to_control_menu(void)
{
    menu_current_state = MENU_STATE_CONTROL;
    ips200_clear();
}

void control_to_ins_menu(void)
{
    menu_current_state = MENU_STATE_INS;
    ips200_clear();
    save_manager_show();
}

void key_handle_ins(void)
{
    static uint8_t last_switch_state = 1; // 上电前拉回高电平
    uint8_t current_switch_state = gpio_get_level(BUTTON_OK);

    uint8_t switch_changed = (last_switch_state != current_switch_state);

    if (switch_changed)
    {
        save_manager_key_ok_pressed();
        last_switch_state = current_switch_state;
        return;
    }

    if (gpio_get_level(BUTTON_DN) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_DN) == 0)
        {
            save_manager_key_down_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_UP) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_UP) == 0)
        {
            save_manager_key_up_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_LF) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_LF) == 0)
        {
            save_manager_key_left_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_RT) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_RT) == 0)
        {
            save_manager_key_right_pressed();
        }
    }
}
