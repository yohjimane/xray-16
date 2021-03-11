#include "pch_script.h"
#include "script_event.h"
#include "xrScriptEngine/ScriptExporter.hpp"

SCRIPT_EXPORT(ScriptEvent, (),
{
    using namespace luabind;

    module(luaState)
    [
        class_<ScriptEvent>("ScriptEvent")
            .def_readwrite("SenderID", &ScriptEvent::SenderID)
            .def_readwrite("Packet", &ScriptEvent::Packet)
    ];
});
