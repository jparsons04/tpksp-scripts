// Version 2 of a helicopter flight script intended for a small helicopter on Duna or a body with a light atmosphere
// Adapted from VTOL hover script with side translation - Ozin, script https://pastebin.com/xeNy4xmv.
// Tested on the Duna Creativity rotor craft in tpksp
// !runscript 
clearscreen.
core:doaction("open terminal",true).
clearvecdraws().
clearguis().
sas off. brakes on.
HUDTEXT("Running script: Helicopter flight for " + ship:name,20,2,22,white,false).

print "Initializing vars".
set spd to 0.
set ht to 0.
set hdg to round(body:geopositionof(ship:facing:forevector):heading,2).

set curPitch to 0.
set curRoll to 0.

set altOverride to 0.
set vsOverride to 0.
set pitchOverride to 0.

set vsMin to choose -6 if altitude < 4500 else -3. set vsMax to 8.
set TAMinSpd to 10. set TAMaxSpd to 35.
set TAHtBuffer to 100.
set maxSpd to 50.
lock autoMaxSpd to max(TAMaxSpd, min(maxSpd, -(altitude-5000)/80 + TAMaxSpd)).

set altErr to 0.
set spdErr to 0.
set vsErr to 0.

// Offset from center of ship to bottom of extended gear is roughly 0.83m for Duna Creativity
set altRadarOffset to -0.83.
set shipBndsRelMax to ship:bounds:relmax:mag.

trackCam off.
terrainDetection on.
enableAutoMaxSpeed on.

avoidingTerrain off.
descending off.
landing off.

lock desHeadingVec to heading(hdg,0):vector * spd.

function setupMenu {
	set telemetryMenu to gui(500, 200). Set telemetryMenu:x to -50. set telemetryMenu:y to 20. telemetryMenu:show. 
	telemetryMenu:addlabel("<b><color=yellow>Helicopter flight telemetry</color></b>").

	set menuLex to lexicon().
	menuLex:add("mDesiredHeader",telemetryMenu:addlabel("==Desired==")).
	menuLex:add("mDesAlt",telemetryMenu:addlabel("Target alt:radar (ht):")).
	menuLex:add("mDesSpd",telemetryMenu:addlabel("Target speed (spd):")).
	menuLex:add("mDesHdg",telemetryMenu:addlabel(".")).
	menuLex:add("mDesVSpd",telemetryMenu:addlabel("Vertical speed override (vsOverride):")).

	menuLex:add("mActualHeader",telemetryMenu:addlabel("==Actual==")).
	menuLex:add("mActAlt",telemetryMenu:addlabel("Altitude:")).
	menuLex:add("mActAlt2",telemetryMenu:addlabel(".")).
	menuLex:add("mActAlt3",telemetryMenu:addlabel(".")).
	menuLex:add("mActGndSpd",telemetryMenu:addlabel("Ground speed:")).
	menuLex:add("mActVSpd",telemetryMenu:addlabel("Vertical speed:")).
	menuLex:add("mActHdg",telemetryMenu:addlabel(".")).

	menuLex:add("mBoundsHeader",telemetryMenu:addlabel("==Bounds==")).
	menuLex:add("mBoundsVSpd",telemetryMenu:addlabel(".")).
	menuLex:add("mBoundsTAHighestNewSpd",telemetryMenu:addlabel(".")).
	menuLex:add("mBoundsMaxSpd",telemetryMenu:addlabel(".")).

	menuLex:add("mOverridesHeader",telemetryMenu:addlabel("==Overrides==")).
	menuLex:add("mOverrideAlt",telemetryMenu:addlabel("altOverride:")).
	menuLex:add("mOverrideVs",telemetryMenu:addlabel("vsOverride:")).
	menuLex:add("mOverridePitch",telemetryMenu:addlabel("pitchOverride:")).

	menuLex:add("mMiscHeader",telemetryMenu:addlabel("==Misc==")).
	menuLex:add("mMiscIPU",telemetryMenu:addlabel(".")).
	menuLex:add("mMiscEC",telemetryMenu:addlabel("Electric Charge:")).
	menuLex:add("mMiscTrackCam",telemetryMenu:addlabel("trackCam:")).
	menuLex:add("mMiscTerrainDetection",telemetryMenu:addlabel("terrainDetection:")).
	menuLex:add("mMiscEnableAutoMaxSpeed",telemetryMenu:addlabel("enableAutoMaxSpeed:")).

	menuLex:add("mErrorsHeader",telemetryMenu:addlabel("==Errors==")).
	menuLex:add("mErrAlt",telemetryMenu:addlabel("Altitude error:")).
	menuLex:add("mErrASpd",telemetryMenu:addlabel("Airspeed error:")).
	menuLex:add("mErrVSpd",telemetryMenu:addlabel("Vertical speed error:")).
	menuLex:add("mErrPitch",telemetryMenu:addlabel("Pitch error:")).

	for txt in telemetryMenu:widgets { set txt:style:fontsize to 14. set txt:style:padding:v to 2. }
	set menuLex["mActualHeader"]:style:margin:top to 15.
	set menuLex["mBoundsHeader"]:style:margin:top to 15.
	set menuLex["mOverridesHeader"]:style:margin:top to 15.
	set menuLex["mMiscHeader"]:style:margin:top to 15.
	set menuLex["mErrorsHeader"]:style:margin:top to 15.
}

print "Initializing rotors".
set topRotors to list().
for m in ship:modulesnamed("ModuleRoboticServoRotor") {
set rotorList to list(m:part). //index 0: part, 1: individual blade partmodules
	for p in m:part:children {
		if p:hasmodule("ModuleControlSurface") {
			rotorList:add(p:getmodule("ModuleControlSurface")).
		}
	}

	if m:part:tag = "top_rotor" {
		topRotors:add(rotorList).
	}
}.

print "Initializing steeringmanager".
set steeringmanager:rollcontrolanglerange to 180.
set steeringmanager:pitchpid:kp to 2.5.
set steeringmanager:rollpid:kp to 0.4.
set steeringmanager:maxstoppingtime to 4.

lock desRoll to choose 0 if groundspeed < 10 else bearingCalculator(body:geopositionof(ship:velocity:surface):heading, hdg)/2.
set p to 0.

print "Initializing PID controllers".
set topAnglePID to pidloop(2.5, 0.3, 2, -3, 15).

set minPitch to -20. set maxPitch to 20.
set desPitchPID to pidloop(2.5, 1, 1).
set desPitchPID:minoutput to minPitch.
set desPitchPID:maxoutput to maxPitch.

set desRollPID to pidloop(1, 1, 1, -3, 3).

print "Setting up blade deploy angle function".
function topDA {
	parameter topAng.
	local desTopDA is topAng.

	for rotor in topRotors {
		for i in range(1,rotor:length) {
			if verticalspeed > -0.5 and verticalspeed < 0.5 and altErr > -0.2 and altErr < 0.2 and airspeed < 5 and alt:radar > 10 {
				// for hovering above ground, set the deploy angle
				// to a value known to be neutral when close to the target alt
				set finalTopDA to 0.
			}
			else {
				set finalTopDA to desTopDA - (p/2).
			}

			rotor[i]:setfield("deploy angle",finalTopDA).
		}
	}
}

print "Setting up utility functions".
function pitchAdjust {
	parameter desPitch.
	local desPitchCopy is desPitch.
	
	if pitchOverride <> 0 {
		set p to pitchOverride.
	}
	else if spd = 0 {
		set p to 0.
	}
	else {
		set p to choose max(minPitch,min(maxPitch,desPitchCopy)) if not(landing) else max(minPitch,min(maxPitch,desPitchCopy/4)).
	}
}

function setTorque {
	parameter tarTorque is 40.
	for module in ship:modulesnamed("ModuleRoboticServoRotor") {
		if module:part:tag = "top_rotor" {
			module:setfield("torque limit(%)", tarTorque).
		}
	}
}

function setRPM {
	parameter tarRPM is 360.
	for module in ship:modulesnamed("ModuleRoboticServoRotor") {
		if module:part:tag = "top_rotor" {
			module:setfield("rpm limit", tarRPM).
		}
	}
}

function horizDistToPos {
	parameter pos.
	return pos:altitudeposition(altitude):mag.
}

function land {
	lock steering to heading(hdg,p).
	set vsOverride to 0.
	set altOverride to round(altitude).
	terrainDetection off. enableAutoMaxSpeed off.
	avoidingTerrain off. descending off.
	landing on.
	
	if groundspeed < 2 {
		print "Setting spd to 0.3".
		set spd to 0.3.
	}
	else {
		print "Decelerating until groundspeed < 2 before setting spd to 0".
		set pitchOverride to 20.

		wait 0.

		when groundspeed < 2 then {
			set pitchOverride to 0.
		}
	}
	
	wait 0.
	
	print "Initiating final landing sequence, then powering down".
	when groundspeed < 2 then {
		set spd to 0.3.
		set altOverride to 0.
		set pitchOverride to 0.
		set ht to 0.6.
		set vsMin to choose -8 if altitude < 4500 else -4.

		when alt:radar+altRadarOffset < 100 then {
			set vsMin to -3.
			set spd to 0.
			gear on.
		}

		on ship:status {
			powerdown().
			set vsMin to -6. set vsMax to 8.
			landing off. descending off.
			terrainDetection on. enableAutoMaxSpeed on.
		}
	}
}

function listAnomalies {
	set as to addons:scansat:getanomalies(body).
	for i in range(as:length) {
		print "as[" + i + "]: " + as[i]:name + " " + round(as[i]:geoposition:distance/1000,2) + "km, hdg: " + round(as[i]:geoposition:heading,2).
	}
}

set keepGetAnomaly to true.
function getAnomaly {
	parameter anomalyIndex is 0, stopDist is 100, startRow is 0.
	set as to addons:scansat:getanomalies(body).
	on time:second {
		print ("as[" + anomalyIndex + "]:geoposition:distance: " + round(horizDistToPos(as[anomalyIndex]:geoposition))):padright(terminal:width) at (0,startRow).
		print ("as[" + anomalyIndex + "]:geoposition:heading: " + round(as[anomalyIndex]:geoposition:heading,2)):padright(terminal:width) at (0,startRow+1).
		print ("as[" + anomalyIndex + "]:geoposition:terrainheight: " + round(as[anomalyIndex]:geoposition:terrainheight)):padright(terminal:width) at (0,startRow+2).
		return as[anomalyIndex]:geoposition:distance > stopDist and keepGetAnomaly.
	}
}

// Function to determine a signed bearing between 2 headings.
function bearingCalculator {
	parameter ref, tgt.
	set rel to tgt - ref.

	if (rel > 180) {
		set rel to rel - 360.
	}
	else if (rel < -180) {
		set rel to rel + 360.
	}

	return rel.
}

function gradualRotation {
	parameter destHdg.
	local bearingDiff is bearingCalculator(body:geopositionof(ship:facing:forevector):heading, destHdg).

	if bearingDiff > 45 {
		until bearingDiff > -1 and bearingDiff < 1 {
			set hdg to hdg + 45.
			set bearingDiff to max(0,bearingDiff - 45).
			
			wait 3.
		}
	}
	else if bearingDiff < -45 {
		until bearingDiff > -1 and bearingDiff < 1 {
			set hdg to hdg - 45.
			set bearingDiff to min(0,bearingDiff + 45).
			
			wait 3.
		}
	}
	else {
		// a desired heading within 45 degrees in either direction is generally a safe rotation
		set hdg to destHdg.
	}
}

set keepGoTo to true.
function goTo {
	parameter destGeoPos, goTospd is 20, goToMaxSpd is maxSpd, startRow is 0.
	
	local destLat is round(destGeoPos:lat,4).
	local destLng is round(destGeoPos:lng,4).
	local destHdg is round(destGeoPos:heading,2).
	
	print "Staring navigation to latlng(" + destLat + ", " + destLng + ")".
	
	print "Aligning to correct heading: " + destHdg.
	gradualRotation(destHdg).
	
	print "Near target heading".
	
	on time:second {
		print ("destination:latlng(" + round(destGeoPos:lat,1) + ", " + round(destGeoPos:lng,1) + ")"):padright(terminal:width) at (0,startRow).
		print ("destination:distance: " + round(horizDistToPos(destGeoPos))):padright(terminal:width) at (0,startRow+1).
		print ("destination:heading: " + round(destGeoPos:heading,2)):padright(terminal:width) at (0,startRow+2).
		print ("destination:terrainheight: " + round(destGeoPos:terrainheight)):padright(terminal:width) at (0,startRow+3).

		// If we drift too far from target heading, realign the steering
		// until about 100m from target, unless we are actively avoiding terrain or descending
		if not(avoidingTerrain) and not(descending) and (hdg-round(destGeoPos:heading,2) < -1 or hdg-round(destGeoPos:heading,2) > 1) and round(horizDistToPos(destGeoPos)) > 100 {
			set hdg to round(destGeoPos:heading,2).
		}

		return keepGoTo and round(horizDistToPos(destGeoPos)) > 20.
	}
	
	set spd to goToSpd.
	set maxSpd to goToMaxSpd.
	
	when round(horizDistToPos(destGeoPos)) < 2000 and keepGoTo then {
		print "Within 2km of destination, slowing to 10m/s, shutting off terrain detection and auto max speed".
		terrainDetection off. enableAutoMaxSpeed off.
		avoidingTerrain off. descending off.

		set spd to 10.
		set pitchOverride to 20.

		wait 0.

		when airspeed < 20 then {
			set pitchOverride to 0.
		}

		when round(horizDistToPos(destGeoPos)) < 100 then {
			print "Close to being over destination".
			print "Current geoposition: " + ship:geoposition.
			print "Preparing to land at destination".
			land().
		}
	}
}

function powerup {
	parameter desiredHeight is 25.
	
	print "Powering up and hovering at alt:radar of " + desiredHeight + "m".
	brakes off.
	settorque(100). setrpm(460).
	set ht to desiredHeight.

	when alt:radar+altRadarOffset >= 0.1 then {
		lock steering to heading(hdg,p,-desRoll).
	}
	
	when alt:radar+altRadarOffset >= 100 then {
		gear off.
	}
}

function powerdown {
	settorque(0). setrpm(460).
	abort on.
	brakes on.
	wait 1.
	abort off.
	on ht powerup(ht).
}

print "Setting up trackCam, panCam".
set keepTrackCam to true.
set cam to addons:camera:flightcamera.
set camPitch to 10.

when trackCam then {
	set cam:position to -ship:facing:forevector:normalized * cam:distance.
	set cam:pitch to camPitch.
	return keepTrackCam.
}

function panCam {
	parameter camPlacement is "star", camDur is 6, camStarPortDist is 20.
	
	set camTimer to time:seconds + camDur.
	
	if camPlacement = "star" or camPlacement <> "port" {
		set camOffset to camStarPortDist.
	}
	else if camPlacement = "port" {
		set camOffset to -camStarPortDist.
	}

	set camGeoPos to body:geopositionof(facing:forevector:normalized * velocity:surface:mag * (camDur/2) + (facing:starvector:normalized * camOffset)).
	set cp to cam:position. 
	trackCam off.

	when true then {
		if camGeoPos:terrainheight >= 0 {
			set cam:position to camGeoPos:position + (facing:topvector:normalized * (alt:radar + 10)).
		}
		else {
			set cam:position to camGeoPos:position + (facing:topvector:normalized * (alt:radar + 10 - camGeoPos:terrainheight)).
		}

		if time:seconds < camTimer + 1 return true.
		else { set cam:position to cp. trackcam on. }
	}
}

print "Setting up terrain avoidance functions".
// The terrain avoidance design here is adapted from the terrain avoidance portion
// of SofieBrink's Plane Lib script https://pastebin.com/pQN2eeTp
set terrainAvoidanceLex to lexicon().
terrainAvoidanceLex:add("highestAlt",0).
terrainAvoidanceLex:add("highestNewSpd",0).
terrainAvoidanceLex:add("highestVSpd",0).

// Function that looks ahead at upcoming terrain and calculates a climbrate override and speed to avoid highest terrain.
function calcTerrainAvoidance {
	local highestTerr is 0.
	local highestSpd is 0.
	local highestVSpd is 0.

	local travel is ship:velocity:surface:normalized.
	for i in range(1, round(airspeed*60), max(1, round(airspeed))) {
		local pos is travel * i.
		local height is max(0, body:geopositionof(pos):terrainheight) + TAHtBuffer + shipBndsRelMax.
		
		if height > highestTerr {
			local heightDiff is max(0.0001,height-altitude).
			
			// As the required altitude increases, we should try to slow down and increase vertical speed in a proportional way.
			// We should also never be going faster than the current speed when actively avoiding terrain.
			local newSpd is min(spd,(spd * (300/heightDiff))).
			local newVSpd is heightDiff / 20.
			
			set highestTerr to height.
			set highestSpd to newSpd.
			set highestVSpd to newVSpd.
		}
	}
	set terrainAvoidanceLex["highestAlt"] to highestTerr.
	set terrainAvoidanceLex["highestNewSpd"] to max(TAMinSpd,min(highestSpd,TAMaxSpd)).
	set terrainAvoidanceLex["highestVSpd"] to highestVSpd.
}

function avoidTerrainOrDescend {
	// When altitude needs to be raised due to terrain, set altOverride to a new safe altitude,
	// set a new speed (equal to or slower than spd) to ensure the new height can be reached,
	// and allow vsMax to be set to a higher value to ensure a safe rate of climb
	if terrainAvoidanceLex["highestAlt"] > altitude + 40 and (altOverride = 0 or terrainAvoidanceLex["highestAlt"] > altOverride + 40) {			
		local newTAAlt is round(terrainAvoidanceLex["highestAlt"]).
		local newTASpd is round(terrainAvoidanceLex["highestNewSpd"],2).
		local newTAMaxVSpd is round(min(12, max(8,terrainAvoidanceLex["highestVSpd"])),1).
		
		print "Setting new values to avoid upcoming terrain:".
		print "altOverride to " + newTAAlt + "m".
		print "spd to " + newTASpd + "m/s".
		print "vsMax to " + newTAMaxVSpd + "m/s".
		
		set altOverride to newTAAlt.
		set spd to newTASpd.

		set vsMax to newTAMaxVSpd. set vsMin to choose -6 if altitude < 4500 else -3.
		
		avoidingTerrain on. descending off.
	}
	// We're at a safe enough altitude above the highest terrain immediately in front of us
	// The terrain ahead is at least 700m below us, so descend to about 500 meters above the highest point
	else if terrainAvoidanceLex["highestAlt"] < altitude - 700 and (altOverride = 0 or terrainAvoidanceLex["highestAlt"] < altOverride - 700) {
		local newTAAlt is round(terrainAvoidanceLex["highestAlt"])+500.

		print "Descending by setting altOverride to " + newTAAlt + "m".
		set altOverride to newTAAlt.

		set vsMax to 8. set vsMin to choose -6 if altitude < 4500 else -3.

		descending on. avoidingTerrain off.
	}
}

set keepTALoop to true.
function terrainAvoidanceLoop {
	local vsMaxCopy is vsMax.
	on round(time:seconds * 2) {
		if terrainDetection and airspeed >= TAMinSpd {
			calcTerrainAvoidance().
			avoidTerrainOrDescend().
		}
		else {
			set terrainAvoidanceLex["highestAlt"] to 0.
			set terrainAvoidanceLex["highestNewSpd"] to 0.
			set terrainAvoidanceLex["highestVSpd"] to 0.
		}

		return keepTALoop.
	}
}

print "Setting up autoMaxSpdLoop function".
set keepAutoMaxSpd to true.
function autoMaxSpdLoop {
	on time:second {
		 if enableAutoMaxSpeed and not(avoidingTerrain) and not(landing) and airspeed >= TAMinSpd-0.5 and spd+3 < autoMaxSpd and (spd-airspeed) < 2 {
			local newAccelSpd is min(maxSpd, round(min(autoMaxSpd,spd + 5),2)).
			print "Accelerating to " + newAccelSpd + "m/s".
			set spd to newAccelSpd.
		}
		
		return keepAutoMaxSpd.
	}
}

print "Setting up mainLoop function".
set keepMainLoop to true.
function mainLoop {
	when true then {
		set desHeadingVecCopy to desHeadingVec.
		set hvel to vxcl(up:vector,velocity:surface).

		set altErr to altitude - (choose altOverride if altOverride <> 0 else ht + max(0,ship:geoposition:terrainheight)).
		set desVSpd to choose vsOverride if vsOverride <> 0 else max(max(vsMin,min(0,-altErr)), min(vsMax,-altErr)).
		
		set vsErr to verticalspeed - desVSpd.
		topDA(topAnglePID:update(time:seconds, vsErr)).
		
		set spdErr to (choose airspeed if vdot(desHeadingVecCopy, hvel) > 0 else -airspeed) - spd.
		pitchAdjust(desPitchPID:update(time:seconds, -spdErr)).
		
		desRollPID:update(time:seconds, desRoll).

		set curPitch to choose vang(ship:facing:topvector, up:vector) if vdot(ship:facing:vector, up:vector) >= 0 else (vang(-facing:topvector, up:vector) - 180).
		if vang(vxcl(ship:facing:vector, up:vector), ship:facing:topvector) < 90 and vang(vcrs(vxcl(up:vector, ship:facing:vector), up:vector), ship:facing:topvector) < 90 {
			set curRoll to -vang(vxcl(ship:facing:vector, up:vector), ship:facing:topvector).
		}
		else {
			set curRoll to vang(vxcl(ship:facing:vector, up:vector), ship:facing:topvector).
		}

		if avoidingTerrain and not(descending) and not(landing) and altErr > -5 {
			print "Near target altitude after avoiding terrain".
			print "Setting vsMax back to 8m/s".
			set vsMax to 8.
			avoidingTerrain off.
		}
		else if descending and not(avoidingTerrain) and not(landing) and altitude-altOverride < 50 {
			print "Near target altitude after descending".
			set vsMin to choose -6 if altitude < 4500 else -3.
			print "Setting vsMin back to " + vsMin + "m/s".
			descending off.
		}

		return keepMainLoop.
	}
}

print "Setting up menu gui display loop".
function showMenu {
	on time:second {
		set menuLex["mDesAlt"]:text to (choose "Target alt:radar (ht): " + ht + "m" if altOverride = 0 else "<color=orange>Altitude override (altOverride):</color> " + altOverride + "m").
		set menuLex["mDesSpd"]:text to "Target speed (spd): " + spd + "m/s".
		set menuLex["mDesHdg"]:text to "Target heading(hdg, p, desRoll): heading(" + round(hdg,2) + ", " + round(p,2) + ", " + round(desRoll,2) + ")".
		set menuLex["mDesVSpd"]:text to (choose "Vertical speed override (vsOverride): N/A" if vsOverride = 0 else "<color=orange>Vertical speed override (vsOverride):</color> " + vsOverride + "m/s").

		set menuLex["mActAlt"]:text to "Altitude: " + round(altitude,2) + "m".
		set menuLex["mActAlt2"]:text to "  alt:radar+altRadarOffset(" + round(alt:radar+altRadarOffset,2) + "m)".
		set menuLex["mActAlt3"]:text to "  terrain height(" + max(0,round(ship:geoposition:terrainheight,2)) + "m)".

		set menuLex["mActGndSpd"]:text to "Ground speed: " + round(groundspeed,2) + "m/s".
		set menuLex["mActVSpd"]:text to "Vertical speed: " + round(verticalspeed,2) + "m/s".
		set menuLex["mActHdg"]:text to "Actual heading: heading(" + round(body:geopositionof(ship:facing:forevector):heading,2) + ", " + round(curPitch,2) + ", " + round(curRoll,2) + ")".

		set menuLex["mBoundsVSpd"]:text to "Vertical speeds: vsMin: " + vsMin + " m/s, vsMax: " + vsMax + "m/s".
		set menuLex["mBoundsTAHighestNewSpd"]:text to (choose "Terrain avoidance speeds: N/A" if not (terrainDetection) else "Terrain avoidance speeds: TAMinSpd: " + TAMinSpd + " m/s, TAMaxSpd: " + TAMaxSpd + "m/s").
		set menuLex["mBoundsMaxSpd"]:text to "maxSpd: " + maxSpd + " m/s".

		set menuLex["mOverrideAlt"]:text to "altOverride: " + altOverride + "m".
		set menuLex["mOverrideVs"]:text to "vsOverride: " + vsOverride + "m/s".
		set menuLex["mOverridePitch"]:text to "pitchOverride: " + pitchOverride + "deg".

		set menuLex["mMiscIPU"]:text to "IPU: " + opcodesleft.
		set menuLex["mMiscEC"]:text to "Electric Charge: " + round(ship:resources[0]:amount,2).
		set menuLex["mMiscTrackCam"]:text to "trackCam: " + trackCam.
		set menuLex["mMiscTerrainDetection"]:text to "terrainDetection: " + terrainDetection.
		set menuLex["mMiscEnableAutoMaxSpeed"]:text to "enableAutoMaxSpeed: " + enableAutoMaxSpeed.

		set menuLex["mErrAlt"]:text to "Altitude error: " + round(altErr,2) + "m".
		set menuLex["mErrASpd"]:text to "Airspeed error: " + round(spdErr,2) + "m/s".
		set menuLex["mErrVSpd"]:text to "Vertical speed error: " + round(vsErr,2) + "m/s".
		set menuLex["mErrPitch"]:text to "Pitch error: " + round(steeringmanager:pitcherror,2) + "deg".
		
		return telemetryMenu:visible.
	}
}

wait 0.

print "Initializing menu gui".
setupMenu().
print "Starting auto max speed loop".
autoMaxSpdLoop().
print "Starting terrain detection and avoidance loop".
terrainAvoidanceLoop().
print "Starting 50Hz main loop".
mainLoop().
print "Displaying menu gui".
showMenu().
powerup().

