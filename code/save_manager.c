#include "save_manager.h"

save_manager_t save_manager_instance;


void save_manager_init(void (*exit_callback)(void))
{
    waypoint_list_init();
    save_manager_instance.active_save = &waypoints;
    save_manager_instance.active_save_idx = 0;
    // 默认先加载0号存档
    bool ret = save_manager_load();
    save_manager_instance.total_saves = 5;
    save_manager_instance.showing_points = 0;
    save_manager_instance.exit_callback = exit_callback;
}

bool save_manager_load(void)
{
    bool ret = waypoint_list_load_from_flash(
        save_manager_instance.active_save,
        INS_FLASH_START_PAGE + save_manager_instance.active_save_idx
    );
    return ret;
}

waypoint_list_t* save_manager_get_active_save(void)
{
    return save_manager_instance.active_save;
}