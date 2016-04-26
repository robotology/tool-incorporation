--
-- Copyright (C) 2012 IITRBCS
-- Authors: Ali Paikan
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--

-- loading lua-yarp binding library
require("yarp")

--
-- create is called when the port monitor is created
-- @return Boolean
--
PortMonitor.create = function(options)
    -- set the constraint here
    print("created!")
    return true;
end

--
-- accept is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Boolean
-- if false is returned, the data will be ignored 
-- and update() will never be called
PortMonitor.accept = function(thing)
    return true
end

--
-- update is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Things
PortMonitor.update = function(thing)
    bb = thing:asBottle()
    blob = bb:get(0):asList()
    local cx = (blob:get(0):asInt() + blob:get(2):asInt()) / 2
    local cy = (blob:get(1):asInt() + blob:get(3):asInt()) / 2	
    bb:clear()
    bb:addInt(cx)
    bb:addInt(cy)
    return thing
end


