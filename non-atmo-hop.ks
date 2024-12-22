// A script to handle a non-atmospheric ballistic hop to a specified GeoCoordinates destination.
// Currently works best when your destination is far away, like, > 50km away)
// Broken up into four phases:
// 1. Initial burn after calculating a launch angle and desired initial velocity
// 2. A re-entry burn when close to the destination to burn off most of the velocity
// 3. A hovering phase to guide the vessel closer to the destination at controlled speeds
// 4. A final landing phase once the vessel is close enough to the destination
// I'd also like future versions of this script to work on bodies with atmospheres.
// !runscript f6JsRKua

burning off. reentry off. hovering off. landing off.

// ==== Variables ====

// Global vars
lock _grav to body:mu/body:position:sqrmagnitude.
lock _maxTWR to ship:availablethrust / ship:mass / _grav.

set bnds to ship:bounds.
lock currentAltitude to bnds:bottomaltradar.

set dest to latlng(0,0).
lock hoverTgt to dest:position.

set destArrow to vecdraw(V(0,0,0),V(0,0,0),green,"destination",3,false,0.3).

set impactLatLng to latlng(0,0).
set impactETA to 0.

debug off.
skipHover off.

// Loop vars
_keepImpact on.
_keepBurningLoop on.
_keepHoverLoop on.
_keepLandingLoop on.

// Hovering vars
set desiredHoverAltitude to 300.
set maxAccelTimeConstant to 0.05.
set maxVelFactor to 0.95.
set minHoverThrottle to 0. set maxHoverThrottle to 1.

lock steeringTopVector to ship:facing:topvector.
set steeringVector to R(0,0,0).

set steeringArrow to vecdraw(V(0,0,0),V(0,0,0),red,"",1,false).

// Landing vars
lock tarLandingDistance to alt:radar - 50.
lock tarLandingSpd to max(tarLandingDistance/4, 0) + 3.

set landingPid to pidloop().
set landingPid:setpoint to -tarLandingSpd.
set landingPid:maxoutput to _maxTWR.
set landingPid:minoutput to 0.

// ==== Initial burn phase helper function ====
// The circle_bearing and circle_distance functions come from KSP-KOS/KSLib's lib_circle_nav
// https://github.com/KSP-KOS/KSLib/blob/master/library/lib_circle_nav.ks

// use to find the initial bearing for the shortest path around a sphere from...
function circle_bearing {
	parameter
	p1, //...this point...
	p2. //...to this point.

	return mod(360+arctan2(sin(p2:lng-p1:lng)*cos(p2:lat),cos(p1:lat)*sin(p2:lat)-sin(p1:lat)*cos(p2:lat)*cos(p2:lng-p1:lng)),360).
}

//use to find the distance from...
function circle_distance {
	parameter
	p1,     //...this point...
	p2,     //...to this point...
	radius. //...around a body of this radius. (note: if you are flying you may want to use ship:body:radius + altitude).

	local A is sin((p1:lat-p2:lat)/2)^2 + cos(p1:lat)*cos(p2:lat)*sin((p1:lng-p2:lng)/2)^2.
 
	return radius*constant():PI*arctan2(sqrt(A),sqrt(1-A))/90.
}

// ==== Re-entry phase helper functions ====
// These helper functions come from KOS-scripts and the "claculated_impact_eta" functions under its Documented Scripts branch
// https://github.com/nuggreat/kOS-scripts/blob/Documented-Scripts/impact%20ETA/claculated_impact_eta.ks

// returns a list of the true anomalies of the 2 points where the craft's orbit passes the given altitude
function alt_to_ta {
	parameter sma, ecc, bodyIn, altIn.

	local rad is altIn + bodyIn:radius.
	local taOfAlt is arccos((-sma * ecc^2 + sma - rad) / (ecc * rad)).
	
	//first true anomaly will be as orbit goes from PE to AP
	return list(taOfAlt,360-taOfAlt).
}

// converts a true anomaly(degrees) to the mean anomaly (degrees) NOTE: only works for non hyperbolic orbits
function ta_to_ma {
	parameter ecc, taDeg.

	local eaDeg is arctan2(sqrt(1-ecc^2) * sin(taDeg), ecc + cos(taDeg)).
	local maDeg is eaDeg - (ecc * sin(eaDeg) * constant:radtodeg).

	return mod(maDeg + 360,360).
}

// returns the difference in time between 2 true anomalies, traveling from taDeg1 to taDeg2
function time_betwene_two_ta {
	parameter ecc, periodIn, taDeg1, taDeg2.
	
	local maDeg1 is ta_to_ma(ecc,taDeg1).
	local maDeg2 is ta_to_ma(ecc,taDeg2).
	
	local timeDiff is periodIn * ((maDeg2 - maDeg1) / 360).
	
	return mod(timeDiff + periodIn, periodIn).
}

// returns the geocoordinates of the ship at a given time(UTs) adjusting for planetary rotation over time, only works for non tilted spin on bodies 
function ground_track {
	parameter pos, posTime, localBody is body.
	
	//using this instead of localBody:north:vector because in many cases the non hard coded value is incorrect
	local bodyNorth is v(0,1,0).

	//the number of degrees the body will rotate in one second
	local rotationalDir is vdot(bodyNorth,localBody:angularvel) * constant:radtodeg.

	local posLATLNG is localBody:geopositionof(pos).
	local timeDif is posTime - time:seconds.
	local longitudeShift is rotationalDir * timeDif.
	local newLNG is mod(posLATLNG:lng + longitudeShift,360).

	if newLNG < - 180 {
		set newLNG to newLNG + 360.
	}

	if newLNG > 180 {
		set newLNG to newLNG - 360.
	}

	return latlng(posLATLNG:lat,newLNG).
}

// returns the Universal Timestamps (UTs) of the ship's impact, NOTE: only works for non hyperbolic orbits
function impact_UTs {
	parameter minError is 1.
	
	if not (defined impact_UTs_impactHeight) {
		set impact_UTs_impactHeight to 0.
	}
	
	local startTime is time:seconds.
	local craftOrbit is ship:obt.
	local sma is craftOrbit:semimajoraxis.
	local ecc is craftOrbit:eccentricity.
	local craftTA is craftOrbit:trueanomaly.
	local orbitPeriod is craftOrbit:period.

	local impactUTs is time_betwene_two_ta(ecc,orbitPeriod,craftTA,alt_to_ta(sma,ecc,body,max(min(impact_UTs_impactHeight,apoapsis - 1),periapsis + 1))[1]) + startTime.
	local newImpactHeight is ground_track(positionat(ship,impactUTs),impactUTs):terrainheight.
	set impact_UTs_impactHeight to (impact_UTs_impactHeight + newImpactHeight) / 2.
	
	//the UTs of the ship's impact, the approximate altitude of the ship's impact, converged will be true when the change in impactHeight between runs is less than the minError
	return lex(
	  "time",impactUTs,
	  "impactHeight",impact_UTs_impactHeight,
	  "converged",((ABS(impact_UTs_impactHeight - newImpactHeight) * 2) < minError)).
}

// _avg_isp and getBurnTime from KSO-KOS lib_navigation
// https://github.com/KSP-KOS/KSLib/blob/master/library/lib_navigation.ks
// KSP-KOS is licensed under the MIT License

// Average Isp calculation
function _avg_isp {
	local burnEngines is list().
	list engines in burnEngines.
	local massBurnRate is 0.
	for e in burnEngines {
		if e:ignition {
			set massBurnRate to massBurnRate + e:availableThrust/(e:ISP * constant:g0).
		}
	}

	local isp is -1.
	
	if massBurnRate <> 0 {
		set isp to ship:availablethrust / massBurnRate.
	}

	return isp.
}

// Burn time from rocket equation
function getBurnTime {
	parameter deltaV.
	parameter isp is 0.
    
	if deltaV:typename = "Vector" {
		set deltaV to deltaV:mag.
	}

	if isp = 0 {
		set isp to _avg_isp().
	}
    
	local burnTime is -1.
	if ship:availablethrust <> 0 {
		set burnTime to ship:mass * (1 - constant:E ^ (-deltaV / isp)) / (ship:availablethrust / isp).
	}

	return burnTime.
}

// ==== Hovering phase helper functions ====
// These functions have been pulled from SofieBrink's Hoverscript V2 script and modified for use here: https://pastebin.com/Euc3wu87

function getAccel {
	// body:atm:altitudepressure without an atmosphere will always return 0
	return ship:availablethrustat(0) / ship:mass.
}

function thrustAt {
	parameter _alt, thr.
	
	return getAccel()*thr - _grav*cos(vang(up:vector, ship:facing:vector)).
}

function getAccelerations {
	// Calculate how much upwards, downwards and sideways acceleration we have available 
	parameter _minThrottle.
	parameter _maxThrottle.
	parameter _desiredAltitude is currentAltitude.
	
	local returnLex is lex().

	returnLex:add("minAccel", min(-0.1,thrustAt(max(currentAltitude, _desiredAltitude), _minThrottle))).
	returnLex:add("maxAccel", max(0.1, thrustAt(min(currentAltitude, _desiredAltitude), _maxThrottle))).
	//returnLex:add("sideAccel", sqrt(max(0.1, max(0.1, ship:availablethrustat(0) * _maxThrottle / ship:mass)^2 - (_grav*1.05)^2))).
	returnLex:add("sideAccel", (max(0.1 , _grav*tan(10)))).

	return returnLex.
}

function getMaxAccelTime {
	// Calculate what the maximum acceleration time should be for the current error in altitude and velocity.
	parameter _desiredAltitude.
	parameter _maxAccelTimeConstant is 0.05.
	local currentScore is choose abs(ship:verticalspeed) + abs(_desiredAltitude - currentAltitude) * _grav
		if (currentAltitude > _desiredAltitude and ship:verticalSpeed < 0) or (currentAltitude < _desiredAltitude and ship:verticalSpeed > 0)
		else abs(ship:verticalspeed) * 2 + abs(_desiredAltitude - currentAltitude) * _grav.
 
	return _maxAccelTimeConstant * max(1, (_maxAccelTimeConstant * 100) / max((_maxAccelTimeConstant * 2.5), currentScore)).
}

function getMaxAttainableVelocity {
	// Calculate from what velocity we could stop in time to reach our desired altitude.
	parameter _desiredAltitude.
	parameter _accelerationLex.
	parameter _maxVelFactor is 0.95.
 
	local returnLex is lex().
 
	returnLex:add("vert",
		choose sqrt(2 * _accelerationLex["minAccel"] * _maxVelFactor * (currentAltitude - _desiredAltitude)) // Calculate for ascending.
		if _desiredAltitude > currentAltitude 
		else -sqrt(2 * _accelerationLex["maxAccel"] * _maxVelFactor * (currentAltitude - _desiredAltitude))). // Calculate for descending.
 
	returnLex:add("side", sqrt(2 * _accelerationLex["sideAccel"] * _maxVelFactor * vxcl(up:vector, hoverTgt):mag)).
 
	return returnLex.
}

function maintainAltitude {
	parameter _desiredAltitude is 0.
	parameter _minThrottle is 0.
	parameter _maxThrottle is 1.
 
	local accelerations is getAccelerations(_minThrottle, _maxThrottle, _desiredAltitude).
	local maxAccelTime is getMaxAccelTime(_desiredAltitude, maxAccelTimeConstant).
	local maxAttainableVelocity is getMaxAttainableVelocity(_desiredAltitude, accelerations, maxVelFactor).
	local desiredAcceleration is (maxAttainableVelocity["vert"] - ship:verticalspeed) / max(0.1, maxAccelTime).
 
	local facingVector is vxcl(up:vector, hoverTgt):normalized*maxAttainableVelocity["side"]/2 - vxcl(vxcl(up:vector, hoverTgt:normalized), vxcl(up:vector, ship:velocity:surface)) * accelerations["sideAccel"] - vxcl(vxcl(vxcl(up:vector, hoverTgt:normalized), vxcl(up:vector, ship:velocity:surface)), vxcl(up:vector, ship:velocity:surface)) * 1.15.

	set steeringVector to lookDirUp(up:vector * _grav * 3 + facingVector:normalized * min(facingVector:mag^0.95, _grav), steeringTopVector).

	local desiredThrottleSetting is _minThrottle + (((desiredAcceleration - accelerations["minAccel"]) * (_maxThrottle - _minThrottle))
		/ max(0.1, (accelerations["maxAccel"] - accelerations["minAccel"]))).
 
	set ship:control:pilotmainthrottle to min(_maxThrottle, desiredThrottleSetting).

	if debug {
		//print "accelerations[sideAccel] " + accelerations["sideAccel"].
		//print "maxAttainableVelocity[side] " + maxAttainableVelocity["side"].
		print "accelerations[maxAccel] - accelerations[minAccel] " + (accelerations["maxAccel"] - accelerations["minAccel"]).
		print "maxAccelTime " + maxAccelTime.
		//print "vxcl(up:vector, hoverTgt):mag " + vxcl(up:vector, hoverTgt):mag.
		//print "facingVector:mag " + facingVector:mag.
		print "desiredThrottleSetting " + desiredThrottleSetting.
		print ".".
	}
}

// ==== Phase loop functions ====

function getImpactLoop {
	on time:seconds {
		if periapsis > 0 {
			print ("No impact detected."):padright(terminal:width) at (0,0).
		}
		else {
			set impactData to impact_UTs().
			set impactLatLng to ground_track(positionat(ship,impactData["time"]),impactData["time"]).
			set impactETA to impactData["time"] - time:seconds.

			set destArrow:startupdater to { return dest:position + up:vector * 10000 * destArrow:scale. }.
			set destArrow:vecupdater to { return -up:vector * 9975. }.
			set destArrow:show to true.

			print ("Distance to destination: " + round(circle_distance(ship:geoposition, dest, body:radius+altitude),2) + "m"):padright(terminal:width) at (0,0).
			print ("Impact ETA: " + round(impactETA,1) + "s"):padright(terminal:width) at (0,1).
			print ("Impact height: " + round(impactData["impactHeight"],2)):padright(terminal:width) at (0,2).
			print ("Impact expected at: latlng(" + round(impactLatLng:lat,2) + "," + round(impactLatLng:lng,2) + ")"):padright(terminal:width) at (0,3).
			
		}
		
		return _keepImpact.
	}
}

function burningLoop {
	parameter distBeyondDest.

	lock distBetweenImpactAndDestination to circle_distance(ship:geoposition, dest, body:radius+altitude) - circle_distance(ship:geoposition, impactLatLng, body:radius+altitude).

	on time:seconds {
		// The order this is evaluated is important
		// Overshoot the impact beyond the destination by just a little bit
		if distBetweenImpactAndDestination < -distBeyondDest {
			print "Impact " + distBeyondDest/1000 + "km beyond destination".
			burning off. reentry on. _keepBurningLoop off.
		}
		else if distBetweenImpactAndDestination < 50e3 {
			lock throttle to 0.2.
		}
		
		return _keepBurningLoop.
	}
}

function hoverLoop {
	parameter dest, landingDistance is 500, manualLand is false.

	on time:seconds {
		if hovering {
			if not steeringmanager:enabled lock steering to steeringVector.
			maintainAltitude(desiredHoverAltitude, minHoverThrottle, maxHoverThrottle).
		}
		else {
			if not (ship:control:pilotmainthrottle = 0) set ship:control:pilotmainthrottle to 0.
			if steeringmanager:enabled unlock steering.
		}

		if debug {
			set steeringArrow:show to true.
			set steeringArrow:vec to (steeringVector:vector:normalized)*30.
		}
		else {
			set steeringArrow:show to false.
		}

		if dest:altitudeposition(altitude):mag < landingDistance and not manualLand {
			print "Within " + landingDistance + "m of destination".
			print "Initiating landing sequence".
			hovering off. landing on. _keepHoverLoop off.
		}
		else if skipHover and not manualLand {
			print "Aborting hovering phase".
			print "Initiating landing sequence".
			hovering off. landing on. _keepHoverLoop off.
		}
		
		return _keepHoverLoop.
	}
}

function landingLoop {
	on time:seconds {
		set landingPid:setpoint to -tarLandingSpd.
		set landingPid:maxoutput to _maxTWR.

		local srfVel is velocity:surface:mag.
		if vdot(velocity:surface, up:vector) < 0 set srfVel to -srfVel.

		local desTWR is landingPid:update(time:seconds, srfVel).
		lock throttle to desTWR/_maxTWR.

		print ("alt:radar: " + round(alt:radar, 1)):padright(terminal:width) at (0,5).
		print ("tgt speed: " + round(tarLandingSpd, 1)):padright(terminal:width) at (0,6).
		print ("max twr:   " + round(_maxTWR, 2)):padright(terminal:width) at (0,7).
		print ("des twr:   " + round(desTWR, 2)):padright(terminal:width) at (0,8).
		print ("velocity:  " + round(srfVel, 1)):padright(terminal:width) at (0,9).

		// On a mostly straight down vertical drop with low ground speed, this guards against the ship
		// chasing the srfretrograde vector which is prone to jumping around at low vertical speeds
		if vang(up:vector, srfretrograde:vector) < 0.5 {
			lock steering to up.
		}

		if ship:status = "landed" {
			lock throttle to 0.
			print "Landed and settling".
			landing off. _keepLandingLoop off. _keepImpact off.
			set destArrow:show to false.
			rcs on. wait 4. rcs off.
			finalReport().
			set ship:control:pilotmainthrottle to 0. lock throttle to 0.
			
			on ship:status {
				unlock steering.
			}
		}
		
		return _keepLandingLoop.
	}
}

// A manual landing function for very short distances
// used after hoverTo() when the parameter manualLand is true
// Best when you select a desiredHoverAltitude that is relatively low, like, < 100m of alt:radar
function hoverLand {
	print "==== Hover Landing ====".
	set desiredHoverAltitude to min(desiredHoverAltitude, 15).
	lock hoverTgt to v(0,0,0).
	gear on.

	when (currentAltitude < desiredHoverAltitude * 1.05 and groundspeed < 2) then {
		local lastupdate is time:seconds.
		on (round(time:seconds*20)) {
			set desiredHoverAltitude to max(-0.5, desiredHoverAltitude - max(desiredHoverAltitude * 0.2, 0.5) * (time:seconds - lastupdate)).
			set lastupdate to time:seconds.
			return not(ship:status = "landed" or ship:status = "splashed").
		}
	}

	on (ship:status = "landed" or ship:status = "splashed") {
		hovering off.
		print "Landed".
		hovering off. landing off. _keepImpact off.
		finalReport().
		set ship:control:pilotmainthrottle to 0. lock throttle to 0.

		on ship:status {
			unlock steering.
		}
	}
}

function calculateLaunch {
	parameter dest, altAboveDestTerrain is 3000.

	set tarR to body:radius+dest:terrainheight+altAboveDestTerrain.
	set pos1 to ship:geoposition:position-body:position.
	set pos2 to dest:position-body:position.

	set angSep to vang(pos1, pos2).
	print "Angle of separation from body's center: " + angSep.
	
	// Further reading on the basis for determining the initial bearing:
	// https://www.movable-type.co.uk/scripts/latlong.html
	set initBrg to circle_bearing(ship:geoposition,dest).
	print "Initial bearing: " + circle_bearing(ship:geoposition,dest).

	// Initial launch angle is dependent on desired eccentricity of the resulting suborbital ellipse
	// Another useful way to visualize surface distance as a function of launch angle and velocity
	// using the same math here
	// https://www.desmos.com/calculator/eqvmftnmcg
	// Notice that the longer the required downrange distance, the shallower the launch angle has to be
	set initLA to arctan((1-sin(angSep/2))/(1+sin(angSep/2))).
	print "Initial launch angle: " + initLA.
	
	// specific orbital energy = -μ/(2a), a = semi major axis of orbit
	// maximum energy when a is minimized, when focus of ellipse is halfway between launch site and destination
	// latus rectum of ellipse = 2*radius * (sin(θ/2)), θ = angSep
	// second focus of ellipse is radius*cos(θ/2) from center of body
	// Ref: https://hopsblog-hop.blogspot.com/2014/06/travel-on-airless-worlds.html
	// a = (radius/2)*(1 + sin(θ/2))
	// deltaV can then be calculated with vis-viva equation
	// v^2 = μ(2/radius - 1/a)
	// Therefore ideal deltaV at each site = sqrt[(2μ/R)·(1-1/{1 + sin(θ/2)})]
	// Reddit discussion where most of this comes from:
	// https://www.reddit.com/r/KerbalAcademy/comments/61tm0f/tool_for_calculating_ballistic_trajectories/
	set tarVel to sqrt((2*body:mu)/tarR * (1-1/(1+sin(angSep/2)))).
	print "Ideal target v0 velocity: " + tarVel.
	
	return lex(
		"initBrg", initBrg,
		"initLA", initLA).
}

function initialBurn {
	parameter altAboveDestTerrain.
	
	// Creating room for the impact display
	print ".". print ".". print ".". print ".". print ".".
	
	local initialParams is calculateLaunch(dest, altAboveDestTerrain).
	local initBrg is initialParams["initBrg"].
	local initLA is initialParams["initLA"].
	
	print "Locking steering to heading(" + initBrg + "," + initLA + ")".
	lock steering to heading(initBrg,initLA).
	
	print "Launching to destination...".
	
	local v0 is getvoice(0).

	from {local i is 3.} until i = 0 step { set i to i-1.} do {
		print(i:tostring).
		v0:play(note("f4", 0.1, 0.5)).
		wait 1.
	}

	print("0").
	v0:play(note("c5", 0.3, 0.5)).
	
	lock throttle to 1.	burning on.
	
	getImpactLoop().
}

function finalReport {
	print "==== Final report ====".
	print "Current geoposition: " + ship:geoposition.
	print "Final distance to destination: " + round(dest:distance,2) + "m".
}

// ==== Main user functions ====

// Ths user can call this function if they are already relatively close to their destination,
// like, within ~5km of their destination
// Best called while landed
function hoverTo {
	parameter _dest, _desiredHoverAltitude is -1, manualLand is false, _landingDistance is 500.

	if _dest:typename <> "GeoCoordinates" {
		print "Destination needs to be a GeoCoordinates object".
	}
	else {
		_keepImpact on.
		_keepBurningLoop on.
		_keepHoverLoop on.
		_keepLandingLoop on.
		skipHover off.

		hovering on.
		set dest to _dest.
		lock hoverTgt to dest:position.
		
		if _desiredHoverAltitude <> -1 {
			set desiredHoverAltitude to _desiredHoverAltitude.
		}
		
		// Creating room for the impact display
		print ".". print ".". print ".". print ".". print ".".
		print "==== Hovering close to destination ====".
		print "Targeting hover altitude of " + round(desiredHoverAltitude) + "m".

		if manualLand {
			print "Manual landing enabled. Run hoverLand() when ready to land.".
		}

		hoverLoop(dest, _landingDistance, manualLand).
		wait 2.
		gear off.

		getImpactLoop().

		when landing and not manualLand then {
			lock throttle to 0.
			lock steering to srfretrograde.
			gear on. rcs on.
			
			print "==== Landing ====".
			print "Steering to srfretrograde".
			
			when vang(ship:facing:vector,srfretrograde:vector) < 5 then {
				rcs off.
				landingLoop().
			}
		}
	}
}

// The primary function that executes all phases for a longer ballistic trajectory.
// The first required argument is expected to be a GeoCoordinates object
function nonAtmoHopTo {
	parameter _dest, altAboveDestTerrain is 3000, distBeyondDest is 2000, reentryBurnBuffer is 3, reentryDeltaVRemaining is 5.

	if _dest:typename <> "GeoCoordinates" {
		print "Destination needs to be a GeoCoordinates object".
	}
	else {
		_keepImpact on.
		_keepBurningLoop on.
		_keepHoverLoop on.
		_keepLandingLoop on.
		skipHover off.

		set dest to _dest.
		lock hoverTgt to dest:position.

		initialBurn(altAboveDestTerrain).

		when burning then {
			print "==== Initial burn ====".
			print "Set destArrow:show to false to turn off destination vecdraw".
			print "To skip the hover phase and force a landing, set skipHover to true".
			
			wait 1.

			gear off.
			burningLoop(distBeyondDest).
		}

		when reentry then {
			print "Cutting throttle and locking steering to srfretrograde".
			lock throttle to 0.
			lock steering to srfretrograde.
			print "==== Re-entry ====".
			
			local etaStartBurn is round(getBurnTime(velocityat(ship,time:seconds+impactETA):surface:mag)+reentryBurnBuffer,2).
			wait 0.
			print "Will begin re-entry burn at impact ETA of " + etaStartBurn + "s".
			print "Will leave " + reentryDeltaVRemaining + "m/s of surface velocity before next phase".

			// Calculate when to start the reentry burn based on the ship's available thrust and the deltav required
			// Again, currently assumes no atmosphere, so this is only calculated once
			// TODO: When implementing an atmospheric launch, this will need to be continuously calculated
			set reqBurnTime to getBurnTime(velocityat(ship,time:seconds+impactETA):surface:mag).
			print ("Required re-entry burn time: " + round(reqBurnTime,2) + "s"):padright(terminal:width) at (0,4).

			when impactETA - 180 > reqBurnTime + reentryBurnBuffer and vang(ship:facing:vector,srfretrograde:vector) < 0.25 then {
				wait 1.
				warpto(time:seconds+impactETA-180).
			}

			when impactETA - 60 > reqBurnTime + reentryBurnBuffer and impactETA - 180 < reqBurnTime + reentryBurnBuffer and vang(ship:facing:vector,srfretrograde:vector) < 0.25 then {
				wait 1.
				warpto(time:seconds+impactETA-60).	
			}
			
			// Give a buffer of reentryBurnBuffer seconds before the required reentry burn
			when (reqBurnTime + reentryBurnBuffer > impactETA) and reentry then {
				when velocity:surface:mag < (reentryDeltaVRemaining*10) then {
					lock throttle to 0.2.
				}

				when velocity:surface:mag < (reentryDeltaVRemaining*5) then {
					lock throttle to 0.15.
				}

				when velocity:surface:mag < reentryDeltaVRemaining then {
					// Done with the re-entry burn
					// Get close to hovering, then transition to hovering phase
					lock throttle to 0.
					set desiredHoverAltitude to currentAltitude+30.
					lock steering to up.
					wait 2.
					lock throttle to (verticalspeed-1)/10.
					wait 1.
					reentry off. hovering on.
				}

				lock throttle to 1.	
			}
		}
		
		when hovering then {
			unlock steering.
			
			if skipHover {
				print "skipHover is true, skipping to landing phase".
				hovering off. landing on.
			}
			else {
				print "==== Hovering close to destination ====".
				print "Targeting hover altitude of " + round(desiredHoverAltitude) + "m".
				wait 1.
				hoverLoop(dest).
			}
		}
		
		when landing then {
			lock throttle to 0.
			lock steering to srfretrograde.
			gear on. rcs on.
			
			// Reset maxhoverthrottle after hovering phase
			set maxhoverthrottle to 1.
			
			print "==== Landing ====".
			print "Steering to srfretrograde".
			
			when vang(ship:facing:vector,srfretrograde:vector) < 1 then {
				rcs off.
				landingLoop().
			}
		}
	}
}

// Licensing attributions:
// KSP-KOS is licensed under the MIT License and some of the functions here come directly from it
// KOS-KOS programs, libraries, program code, and examples of program code are free software.
// Permission is granted to change, share, and use them (with minor restrictions), even privately or commercially, under the terms of the MIT license (https://github.com/KSP-KOS/KSLib/blob/master/LICENSE).

clearscreen.

