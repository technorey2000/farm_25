idf_component_register(SRCS "farm_25_main.c"
"./src/Shared" 

"./src/Task/dispatcherTask.c"
"./src/Task/serialTask.c"
"./src/Task/sysControlTask.c"
"./src/Task/stateMachines.c"
"./src/Task/bleTask.c"
"./src/Task/strgTask.c"
"./src/Task/sensorsTask.c"

"./src/Sync/syncTask.c"

"./src/Shared/messages.c"
"./src/Shared/cJSON.c"
"./src/Shared/spiffs_circular_queue.c"

"./components/dht/dht11.c"


INCLUDE_DIRS "." "./include"
)
