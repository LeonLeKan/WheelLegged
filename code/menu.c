#include "menu.h"
#include "image.h"
#include "imu.h"
#include "screen_interface.h"
#include "save_manager_ui.h"
#include "pid.h"

volatile menu_state_t menu_current_state = MENU_STATE_CONTROL;

MENU *WheelLeg, *Balance, *Image, *Element, *testnum, *road2, *modiro, *thnum, *tunum, *uturn3, *jump, *saveall, *currentMenu;
MENU *speed, *anloop1, *anloop2, *sploop, *sploop2, *tuloop1, *tuloop2, *asloop, *snloop, *yaloop, *josloop, *joyloop, *imubias, *pzero, *pinit,
    *esha, *imexp, *farl, *nearl, *stdis1, *stdis2, *stdis3, *kgray, *whiga, *minth, *maxth, *sicnt, *frcnt, *bacnt, *jocnt, *saveim,
    *change, *snspeed, *rollba, *turnamf, *turnamt, *cwtucop, *ccwtucop, *stspeed, *jospeed, *snlock, *rospeed, *turnamj, *baspeed, *joangle, *jolock, *xythun, *tunext, *dis1, *speedl, *balspeed, *thsp, *rotusp, *dis2, *curnext, *tunesp, *cwtucop1, *ccwtucop1, *uturnsp, *saveel;
MENU *an1kp, *an1ki, *an1kd,
    *an2kp, *an2ki, *an2kd,
    *spkp, *spki, *spkd,
    *sp2kp, *sp2ki, *sp2kd,
    *tu1kp, *tu1ki, *tu1kd,
    *tu2kp, *tu2ki, *tu2kd,
    *askp, *aski, *askd,
    *snkp, *snki, *snkd,
    *yakp, *yaki, *yakd,
    *joskp, *joski, *joskd,
    *joykp, *joyki, *joykd,
    *rollbias, *pitchbias, *yawbias,
    *zeroan1, *zeroan2, *zerosp, *zerosp2, *zerotu1, *zerotu2, *zeroas, *zerosn, *zeroya, *zerojos, *zerojoy, *zerobi,
    *savean1, *savean2, *savesp, *savesp2, *savetu1, *savetu2, *saveas, *savesn, *saveya, *savejos, *savejoy, *savebi;

#define MENU_POOL_SIZE 256

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
    WheelLeg = createMenu("WheelLeg", 0, 1, MENU_TYPE_NORMAL, NULL);
    Balance = createMenu("Balance", 1, 1, MENU_TYPE_NORMAL, NULL);
    Image = createMenu("Image", 1, 2, MENU_TYPE_NORMAL, NULL);
    Element = createMenu("Element", 1, 3, MENU_TYPE_NORMAL, NULL);
    testnum = createMenu("testnum", 1, 4, MENU_TYPE_INT, NULL);
    road2 = createMenu("road2", 1, 5, MENU_TYPE_INT, NULL);
    modiro = createMenu("modiro", 1, 6, MENU_TYPE_INT, NULL);
    thnum = createMenu("thnum", 1, 7, MENU_TYPE_INT, NULL);
    tunum = createMenu("tunum", 1, 8, MENU_TYPE_INT, NULL);
    uturn3 = createMenu("uturn3", 1, 9, MENU_TYPE_INT, NULL);
    jump = createMenu("jump", 1, 10, MENU_TYPE_INT, NULL);
    saveall = createMenu("save", 1, 11, MENU_TYPE_FUNCTION, menu_save);

    WheelLeg->child = Balance;
    Balance->parent = WheelLeg;
    Image->parent = WheelLeg;
    Element->parent = WheelLeg;
    testnum->parent = WheelLeg;
    road2->parent = WheelLeg;
    modiro->parent = WheelLeg;
    thnum->parent = WheelLeg;
    tunum->parent = WheelLeg;
    uturn3->parent = WheelLeg;
    jump->parent = WheelLeg;
    saveall->parent = WheelLeg;

    Balance->next = Image;
    Image->last = Balance;
    Image->next = Element;
    Element->last = Image;
    Element->next = testnum;
    testnum->last = Element;
    testnum->next = road2;
    road2->last = testnum;
    road2->next = modiro;
    modiro->last = road2;
    modiro->next = thnum;
    thnum->last = modiro;
    thnum->next = tunum;
    tunum->last = thnum;
    tunum->next = uturn3;
    uturn3->last = tunum;
    uturn3->next = jump;
    jump->last = uturn3;
    jump->next = saveall;
    saveall->last = jump;
    saveall->next = Balance;
    Balance->last = saveall;

    currentMenu = WheelLeg->child;
}

void create_2nd_menu(void)
{
    speed = createMenu("speed", 2, 1, MENU_TYPE_INT, NULL);
    anloop1 = createMenu("anloop1", 2, 2, MENU_TYPE_NORMAL, NULL);
    anloop2 = createMenu("anloop2", 2, 3, MENU_TYPE_NORMAL, NULL);
    sploop = createMenu("sploop", 2, 4, MENU_TYPE_NORMAL, NULL);
    sploop2 = createMenu("sploop2", 2, 5, MENU_TYPE_NORMAL, NULL);
    tuloop1 = createMenu("tuloop1", 2, 6, MENU_TYPE_NORMAL, NULL);
    tuloop2 = createMenu("tuloop2", 2, 7, MENU_TYPE_NORMAL, NULL);
    asloop = createMenu("asloop", 2, 8, MENU_TYPE_NORMAL, NULL);
    snloop = createMenu("snloop", 2, 9, MENU_TYPE_NORMAL, NULL);
    yaloop = createMenu("yaloop", 2, 10, MENU_TYPE_NORMAL, NULL);
    josloop = createMenu("josloop", 2, 11, MENU_TYPE_NORMAL, NULL);
    joyloop = createMenu("joyloop", 2, 12, MENU_TYPE_NORMAL, NULL);
    imubias = createMenu("imubias", 2, 13, MENU_TYPE_NORMAL, NULL);
    pzero = createMenu("pzero", 2, 14, MENU_TYPE_FLOAT, NULL);
    pinit = createMenu("pinit", 2, 15, MENU_TYPE_FLOAT, NULL);

    esha = createMenu("esha", 2, 1, MENU_TYPE_INT, NULL);
    imexp = createMenu("imexp", 2, 2, MENU_TYPE_INT, NULL);
    farl = createMenu("farl", 2, 3, MENU_TYPE_INT, NULL);
    nearl = createMenu("nearl", 2, 4, MENU_TYPE_INT, NULL);
    stdis1 = createMenu("stdis1", 2, 5, MENU_TYPE_INT, NULL);
    stdis2 = createMenu("stdis2", 2, 6, MENU_TYPE_INT, NULL);
    stdis3 = createMenu("stdis3", 2, 7, MENU_TYPE_INT, NULL);
    kgray = createMenu("kgray", 2, 8, MENU_TYPE_INT, NULL);
    whiga = createMenu("whiga", 2, 9, MENU_TYPE_INT, NULL);
    minth = createMenu("minth", 2, 10, MENU_TYPE_INT, NULL);
    maxth = createMenu("maxth", 2, 11, MENU_TYPE_INT, NULL);
    sicnt = createMenu("sicnt", 2, 12, MENU_TYPE_INT, NULL);
    frcnt = createMenu("frcnt", 2, 13, MENU_TYPE_INT, NULL);
    bacnt = createMenu("bacnt", 2, 14, MENU_TYPE_INT, NULL);
    jocnt = createMenu("jocnt", 2, 15, MENU_TYPE_INT, NULL);
    saveim = createMenu("save", 2, 16, MENU_TYPE_FUNCTION, menu_save);

    change = createMenu("change", 2, 1, MENU_TYPE_FUNCTION, control_to_ins_menu);
    snspeed = createMenu("snspeed", 2, 2, MENU_TYPE_INT, NULL);
    rollba = createMenu("rollba", 2, 3, MENU_TYPE_FLOAT, NULL);
    turnamf = createMenu("turnamf", 2, 4, MENU_TYPE_INT, NULL);
    turnamt = createMenu("turnamt", 2, 5, MENU_TYPE_INT, NULL);
    cwtucop = createMenu("cwtucop", 2, 6, MENU_TYPE_FLOAT, NULL);
    ccwtucop = createMenu("ccwtucop", 2, 7, MENU_TYPE_FLOAT, NULL);
    stspeed = createMenu("stspeed", 2, 8, MENU_TYPE_INT, NULL);
    jospeed = createMenu("jospeed", 2, 9, MENU_TYPE_INT, NULL);
    snlock = createMenu("snlock", 2, 10, MENU_TYPE_INT, NULL);
    rospeed = createMenu("rospeed", 2, 11, MENU_TYPE_INT, NULL);
    turnamj = createMenu("turnamj", 2, 12, MENU_TYPE_INT, NULL);
    baspeed = createMenu("baspeed", 2, 13, MENU_TYPE_INT, NULL);
    joangle = createMenu("joangle", 2, 14, MENU_TYPE_FLOAT, NULL);
    jolock = createMenu("jolock", 2, 15, MENU_TYPE_INT, NULL);
    xythun = createMenu("xythun", 2, 16, MENU_TYPE_FLOAT, NULL);
    tunext = createMenu("tunext", 2, 17, MENU_TYPE_INT, NULL);
    dis1 = createMenu("dis1", 2, 18, MENU_TYPE_FLOAT, NULL);
    speedl = createMenu("speedl", 2, 19, MENU_TYPE_INT, NULL);
    balspeed = createMenu("balspeed", 2, 20, MENU_TYPE_INT, NULL);
    thsp = createMenu("thsp", 2, 21, MENU_TYPE_INT, NULL);
    rotusp = createMenu("rotusp", 2, 22, MENU_TYPE_INT, NULL);
    dis2 = createMenu("dis2", 2, 23, MENU_TYPE_FLOAT, NULL);
    curnext = createMenu("curnext", 2, 24, MENU_TYPE_INT, NULL);
    tunesp = createMenu("tunesp", 2, 25, MENU_TYPE_INT, NULL);
    cwtucop1 = createMenu("cwtucop1", 2, 26, MENU_TYPE_FLOAT, NULL);
    ccwtucop1 = createMenu("ccwtucop1", 2, 27, MENU_TYPE_FLOAT, NULL);
    uturnsp = createMenu("uturnsp", 2, 28, MENU_TYPE_INT, NULL);
    saveel = createMenu("save", 2, 29, MENU_TYPE_FUNCTION, menu_save);

    speed->next = anloop1;
    anloop1->last = speed;
    anloop1->next = anloop2;
    anloop2->last = anloop1;
    anloop2->next = sploop;
    sploop->last = anloop2;
    sploop->next = sploop2;
    sploop2->last = sploop;
    sploop2->next = tuloop1;
    tuloop1->last = sploop2;
    tuloop1->next = tuloop2;
    tuloop2->last = tuloop1;
    tuloop2->next = asloop;
    asloop->last = tuloop2;
    asloop->next = snloop;
    snloop->last = asloop;
    snloop->next = yaloop;
    yaloop->last = snloop;
    yaloop->next = josloop;
    josloop->last = yaloop;
    josloop->next = joyloop;
    joyloop->last = josloop;
    joyloop->next = imubias;
    imubias->last = joyloop;
    imubias->next = pzero;
    pzero->last = imubias;
    pzero->next = pinit;
    pinit->last = pzero;
    pinit->next = speed;
    speed->last = pinit;

    esha->next = imexp;
    imexp->last = esha;
    imexp->next = farl;
    farl->last = imexp;
    farl->next = nearl;
    nearl->last = farl;
    nearl->next = stdis1;
    stdis1->last = nearl;
    stdis1->next = stdis2;
    stdis2->last = stdis1;
    stdis2->next = stdis3;
    stdis3->last = stdis2;
    stdis3->next = kgray;
    kgray->last = stdis3;
    kgray->next = whiga;
    whiga->last = kgray;
    whiga->next = minth;
    minth->last = whiga;
    minth->next = maxth;
    maxth->last = minth;
    maxth->next = sicnt;
    sicnt->last = maxth;
    sicnt->next = frcnt;
    frcnt->last = sicnt;
    frcnt->next = bacnt;
    bacnt->last = frcnt;
    bacnt->next = jocnt;
    jocnt->last = bacnt;
    jocnt->next = saveim;
    saveim->last = jocnt;
    saveim->next = esha;
    esha->last = saveim;

    change->next = snspeed;
    snspeed->last = change;
    snspeed->next = rollba;
    rollba->last = snspeed;
    rollba->next = turnamf;
    turnamf->last = rollba;
    turnamf->next = turnamt;
    turnamt->last = turnamf;
    turnamt->next = cwtucop;
    cwtucop->last = turnamt;
    cwtucop->next = ccwtucop;
    ccwtucop->last = cwtucop;
    ccwtucop->next = stspeed;
    stspeed->last = ccwtucop;
    stspeed->next = jospeed;
    jospeed->last = stspeed;
    jospeed->next = snlock;
    snlock->last = jospeed;
    snlock->next = rospeed;
    rospeed->last = snlock;
    rospeed->next = turnamj;
    turnamj->last = rospeed;
    turnamj->next = baspeed;
    baspeed->last = turnamj;
    baspeed->next = joangle;
    joangle->last = baspeed;
    joangle->next = jolock;
    jolock->last = joangle;
    jolock->next = xythun;
    xythun->last = jolock;
    xythun->next = tunext;
    tunext->last = xythun;
    tunext->next = dis1;
    dis1->last = tunext;
    dis1->next = speedl;
    speedl->last = dis1;
    speedl->next = balspeed;
    balspeed->last = speedl;
    balspeed->next = thsp;
    thsp->last = balspeed;
    thsp->next = rotusp;
    rotusp->last = thsp;
    rotusp->next = dis2;
    dis2->last = rotusp;
    dis2->next = curnext;
    curnext->last = dis2;
    curnext->next = tunesp;
    tunesp->last = curnext;
    tunesp->next = cwtucop1;
    cwtucop1->last = tunesp;
    cwtucop1->next = ccwtucop1;
    ccwtucop1->last = cwtucop1;
    ccwtucop1->next = uturnsp;
    uturnsp->last = ccwtucop1;
    uturnsp->next = saveel;
    saveel->last = uturnsp;
    saveel->next = change;
    change->last = saveel;

    Balance->child = speed;
    speed->parent = Balance;
    anloop1->parent = Balance;
    anloop2->parent = Balance;
    sploop->parent = Balance;
    sploop2->parent = Balance;
    tuloop1->parent = Balance;
    tuloop2->parent = Balance;
    asloop->parent = Balance;
    snloop->parent = Balance;
    yaloop->parent = Balance;
    josloop->parent = Balance;
    joyloop->parent = Balance;
    imubias->parent = Balance;
    pzero->parent = Balance;
    pinit->parent = Balance;

    Image->child = esha;
    esha->parent = Image;
    imexp->parent = Image;
    farl->parent = Image;
    nearl->parent = Image;
    stdis1->parent = Image;
    stdis2->parent = Image;
    stdis3->parent = Image;
    kgray->parent = Image;
    whiga->parent = Image;
    minth->parent = Image;
    maxth->parent = Image;
    sicnt->parent = Image;
    frcnt->parent = Image;
    bacnt->parent = Image;
    jocnt->parent = Image;
    saveim->parent = Image;

    Element->child = change;
    change->parent = Element;
    snspeed->parent = Element;
    rollba->parent = Element;
    turnamf->parent = Element;
    turnamt->parent = Element;
    cwtucop->parent = Element;
    ccwtucop->parent = Element;
    stspeed->parent = Element;
    jospeed->parent = Element;
    snlock->parent = Element;
    rospeed->parent = Element;
    turnamj->parent = Element;
    baspeed->parent = Element;
    joangle->parent = Element;
    jolock->parent = Element;
    xythun->parent = Element;
    tunext->parent = Element;
    dis1->parent = Element;
    speedl->parent = Element;
    balspeed->parent = Element;
    thsp->parent = Element;
    rotusp->parent = Element;
    dis2->parent = Element;
    curnext->parent = Element;
    tunesp->parent = Element;
    cwtucop1->parent = Element;
    ccwtucop1->parent = Element;
    uturnsp->parent = Element;
    saveel->parent = Element;
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

    sp2kp = createMenu("sp2kp", 3, 1, MENU_TYPE_FLOAT, NULL);
    sp2ki = createMenu("sp2ki", 3, 2, MENU_TYPE_FLOAT, NULL);
    sp2kd = createMenu("sp2kd", 3, 3, MENU_TYPE_FLOAT, NULL);

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

    yakp = createMenu("yakp", 3, 1, MENU_TYPE_FLOAT, NULL);
    yaki = createMenu("yaki", 3, 2, MENU_TYPE_FLOAT, NULL);
    yakd = createMenu("yakd", 3, 3, MENU_TYPE_FLOAT, NULL);

    joskp = createMenu("joskp", 3, 1, MENU_TYPE_FLOAT, NULL);
    joski = createMenu("joski", 3, 2, MENU_TYPE_FLOAT, NULL);
    joskd = createMenu("joskd", 3, 3, MENU_TYPE_FLOAT, NULL);

    joykp = createMenu("joykp", 3, 1, MENU_TYPE_FLOAT, NULL);
    joyki = createMenu("joyki", 3, 2, MENU_TYPE_FLOAT, NULL);
    joykd = createMenu("joykd", 3, 3, MENU_TYPE_FLOAT, NULL);

    rollbias = createMenu("rollbias", 3, 1, MENU_TYPE_FLOAT, NULL);
    pitchbias = createMenu("pitchbias", 3, 2, MENU_TYPE_FLOAT, NULL);
    yawbias = createMenu("yawbias", 3, 3, MENU_TYPE_FLOAT, NULL);

    savean1 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savean2 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savesp = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savesp2 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savetu1 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savetu2 = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    saveas = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savesn = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    saveya = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savejos = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savejoy = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);
    savebi = createMenu("save", 3, 4, MENU_TYPE_FUNCTION, menu_save);

    zeroan1 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zeroan2 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerosp = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerosp2 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerotu1 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerotu2 = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zeroas = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerosn = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zeroya = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerojos = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerojoy = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);
    zerobi = createMenu("zero", 3, 5, MENU_TYPE_FUNCTION, menu_zero);

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

    sploop2->child = sp2kp;
    sp2kp->next = sp2ki;
    sp2ki->last = sp2kp;
    sp2ki->next = sp2kd;
    sp2kd->last = sp2ki;
    sp2kd->next = savesp2;
    savesp2->last = sp2kd;
    savesp2->next = zerosp2;
    zerosp2->last = savesp2;
    zerosp2->next = sp2kp;
    sp2kp->last = zerosp2;

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

    yaloop->child = yakp;
    yakp->next = yaki;
    yaki->last = yakp;
    yaki->next = yakd;
    yakd->last = yaki;
    yakd->next = saveya;
    saveya->last = yakd;
    saveya->next = zeroya;
    zeroya->last = saveya;
    zeroya->next = yakp;
    yakp->last = zeroya;

    josloop->child = joskp;
    joskp->next = joski;
    joski->last = joskp;
    joski->next = joskd;
    joskd->last = joski;
    joskd->next = savejos;
    savejos->last = joskd;
    savejos->next = zerojos;
    zerojos->last = savejos;
    zerojos->next = joskp;
    joskp->last = zerojos;

    joyloop->child = joykp;
    joykp->next = joyki;
    joyki->last = joykp;
    joyki->next = joykd;
    joykd->last = joyki;
    joykd->next = savejoy;
    savejoy->last = joykd;
    savejoy->next = zerojoy;
    zerojoy->last = savejoy;
    zerojoy->next = joykp;
    joykp->last = zerojoy;

    imubias->child = rollbias;
    rollbias->next = pitchbias;
    pitchbias->last = rollbias;
    pitchbias->next = yawbias;
    yawbias->last = pitchbias;
    yawbias->next = savebi;
    savebi->last = yawbias;
    savebi->next = zerobi;
    zerobi->last = savebi;
    zerobi->next = rollbias;
    rollbias->last = zerobi;

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

    sp2kp->parent = sploop2;
    sp2ki->parent = sploop2;
    sp2kd->parent = sploop2;
    zerosp2->parent = sploop2;
    savesp2->parent = sploop2;

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

    yakp->parent = yaloop;
    yaki->parent = yaloop;
    yakd->parent = yaloop;
    zeroya->parent = yaloop;
    saveya->parent = yaloop;

    joskp->parent = josloop;
    joski->parent = josloop;
    joskd->parent = josloop;
    zerojos->parent = josloop; 
    savejos->parent = josloop;

    joykp->parent = joyloop;
    joyki->parent = joyloop;
    joykd->parent = joyloop;
    zerojoy->parent = joyloop;
    savejoy->parent = joyloop;

    rollbias->parent = imubias;
    pitchbias->parent = imubias;
    yawbias->parent = imubias;
    zerobi->parent = imubias;
    savebi->parent = imubias;
}

void key_handle_menu(void)
{
    if (!edit_mode)
    {
        if (gpio_get_level(BUTTON_OK) == 0)
        {
            system_delay_ms(10);
            ips200_clear();
            if (gpio_get_level(BUTTON_OK) == 0)
            {
                if (currentMenu->type == MENU_TYPE_FUNCTION && currentMenu->Function != NULL)
                {
                    currentMenu->Function();
                    if (currentMenu != change)
                    {
                        currentMenu->exec_flag = 1;
                        ips200_clear();
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
                        sprintf(edit_buf, "%+09.4f", currentMenu->data_float);
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
                sprintf(buf, "%+09.4f", currentMenu->data_float);
            }
            pstr = buf;
        }

        for (int i = 0; i < strlen(pstr); i++)
        {
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
        sprintf(buf_float_kp, "%+09.4f", kp_node->data_float);
        sprintf(buf_float_ki, "%+09.4f", ki_node->data_float);
        sprintf(buf_float_kd, "%+09.4f", kd_node->data_float);
    }
    else
    {
        sprintf(buf_float_kp, "%+09.4f", 0.0);
        sprintf(buf_float_ki, "%+09.4f", 0.0);
        sprintf(buf_float_kd, "%+09.4f", 0.0);
    }

    ips200_show_string(168, 192, buf_float_kp);
    ips200_show_string(168, 208, buf_float_ki);
    ips200_show_string(168, 224, buf_float_kd);
}

void menu_save(void)
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
        } while (tmp != img);
    }

    MENU *ele = Element->child;
    if (ele != NULL)
    {
        MENU *tmp = ele;
        do
        {
            if (tmp->type == MENU_TYPE_INT)
            {
                flash_union_buffer[index++].int32_type = tmp->data_int;
                node_count++;
            }
            else if(tmp->type == MENU_TYPE_FLOAT)
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
        } while (tmp != ele);
    }

    MENU *test = testnum;
    flash_union_buffer[index++].int32_type = test->data_int;
    node_count++;

    MENU *road = road2;
    flash_union_buffer[index++].int32_type = road->data_int;
    node_count++;

    MENU *modify_road = modiro;
    flash_union_buffer[index++].int32_type = modify_road->data_int;
    node_count++;

    MENU *thun_num = thnum;
    flash_union_buffer[index++].int32_type = thun_num->data_int;
    node_count++;

    MENU *turn_num = tunum;
    flash_union_buffer[index++].int32_type = turn_num->data_int;
    node_count++;

    MENU *uturn_3 = uturn3;
    flash_union_buffer[index++].int32_type = uturn_3->data_int;
    node_count++;

    MENU *jumpmode = jump;
    flash_union_buffer[index++].int32_type = jumpmode->data_int;
    node_count++;

    flash_union_buffer[0].float_type = menu_magic;
    flash_union_buffer[1].uint32_type = node_count;

    flash_erase_page(0, MENU1_FLASH_PAGE);
    flash_write_page_from_buffer(0, MENU1_FLASH_PAGE, index);
}

void menu_load(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, MENU1_FLASH_PAGE, FLASH_PAGE_LENGTH);

    if (flash_union_buffer[0].float_type != menu_magic)
        return;

    uint32_t idx = 2;

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
            }
            tmp = tmp->next;
        } while (tmp != img);
    }

    MENU *ele = Element->child;
    if (ele != NULL)
    {
        MENU *tmp = ele;
        do
        {
            if (tmp->type == MENU_TYPE_INT)
            {
                tmp->data_int = (int)flash_union_buffer[idx++].int32_type;
                if (fabs((float)tmp->data_int) >= 100000)
                {
                    tmp->data_int = 0;
                }
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
            }
            tmp = tmp->next;
        } while (tmp != ele);
    }

    MENU *test = testnum;
    test->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)test->data_int) >= 100000)
    {
        test->data_int = 0;
    }

    MENU *road = road2;
    road->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)road->data_int) >= 100000)
    {
        road->data_int = 0;
    }

    MENU *modify_road = modiro;
    modify_road->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)modify_road->data_int) >= 100000)
    {
        modify_road->data_int = 0;
    }

    MENU *thun_num = thnum;
    thun_num->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)thun_num->data_int) >= 100000)
    {
        thun_num->data_int = 0;
    }

    MENU *turn_num = tunum;
    turn_num->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)turn_num->data_int) >= 100000)
    {
        turn_num->data_int = 0;
    }

    MENU *uturn_3 = uturn3;
    uturn_3->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)uturn_3->data_int) >= 100000)
    {
        uturn_3->data_int = 0;
    }

    MENU *jumpmode = jump;
    jumpmode->data_int = (int)flash_union_buffer[idx++].int32_type;
    if (fabs((float)jumpmode->data_int) >= 100000)
    {
        jumpmode->data_int = 0;
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

void ins_to_control_menu(void)
{
    menu_current_state = MENU_STATE_CONTROL;
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
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
    if (gpio_get_level(BUTTON_OK) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_OK) == 0)
        {
            save_manager_key_ok_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_DN) == 0)
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
