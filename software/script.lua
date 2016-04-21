--script.lua
dofile("lang.lua")
dofile("util.lua")
dofile("util.lua")
dofile("HIDview.lua")
require("canview")
mdiArea:addSubWindow( CanView(0x0483, 0x5750)  ):show()