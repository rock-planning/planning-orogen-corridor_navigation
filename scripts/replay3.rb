#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end

logdir = ARGV[0]

#load log file 
log = Orocos::Log::Replay.open(File.join( logdir, "properties.0.log" ), 
				File.join( logdir, "dynamixel.0.log" ),
				File.join( logdir, "lowlevel.0.log" ),
				File.join( logdir, "hokuyo.0.log" ),
				File.join( logdir, "eslam_pose_estimator.0.log" ),
				File.join( logdir, "eslam_mapping.0.log" ),
				File.join( logdir, "selected_corridors.0.log" ))

Orocos::CORBA::max_message_size = 100000000
Bundles.initialize
Nameservice::enable(:Local)
#Vizkit::ReaderWriterProxy.default_policy= Hash.new

Bundles.run "corridorNavigationTest", :valgrind => false, :output => nil do |p|
    corridor_servoing = Bundles.get('corridor_servoing')

    ls = nil
    log.filter.filtered_scans.connect_to( corridor_servoing.scan_samples, :type => :buffer, :size => 1000 ) do |s|
	ls = s
    end
    log.odometry.odometry_samples.connect_to( corridor_servoing.odometry_samples, :type => :buffer, :size => 1000 ) 

    Nameservice::Local.registered_tasks["odometry"] = log.odometry
    Nameservice::Local.registered_tasks["dynamixel"] = log.dynamixel

    Orocos.conf.apply( corridor_servoing, ['default'], true )
    #Orocos.conf.apply( eslam, ['default', 'localisation'], true )
    
#     log.odometry.odometry_samples.connect_to( corridor_servoing.dynamic_transformations, :type => :buffer, :size => 1000 ) 
#     log.asguard_simulation.lowerDynamixel2UpperDynamixel.connect_to( corridor_servoing.dynamic_transformations, :type => :buffer, :size => 1000 ) 
#     log.odometry.odometry_samples.connect_to( corridor_servoing.dynamic_transformations, :type => :buffer, :size => 1000 ) 
    
    Bundles.transformer.setup( corridor_servoing )
    
    corridor_servoing.configure
    
    view3d = Vizkit.vizkit3d_widget
    mapview = view3d.createPlugin('vfh_star', 'TraversabilityMapGeneratorVisualization')
    treeview = view3d.createPlugin('vfh_star', 'VFHTreeVisualization')
    planview = view3d.createPlugin('corridor_planner', 'CorridorPlanVisualization')
    posview = view3d.createPlugin('vizkit-base', 'RigidBodyStateVisualization')
    view3d.show()
    
    map = nil
    pose = nil
    map_id = '/slam/mls'
    
    odo_sample = nil
    corridor = nil
    cnt = 0
    hd_writer = corridor_servoing.heading.writer()
    
    log.eslam_mapping.map_id do |s|
	map_id = s
	s
    end
        
    log.eslam_mapping.map do |sample|
	map = sample
	sample
    end

    log.odometry.odometry_samples do |sample|
	odo_sample = sample
	sample
    end
    
    log.controller.selected_corridors do |sample|
	corridor = sample.corridors[0]
	planview.updateData(sample)
	sample
    end

    log.eslam_pose_estimator.pose_samples do |sample|
	posview.updateData(sample)
	cur_map_pose = sample
	if(map)
	    puts("got new map, restarting")
	    if(corridor_servoing.state == :RUNNING || corridor_servoing.state == :EXCEPTION)
		corridor_servoing.stop
		while(corridor_servoing.state == :STOPPED)
		    puts("Waiting for task to stop")
		end
	    end
	    corridor_servoing.setMap(map, map_id, cur_map_pose)
	    map = nil
	    
	    corridor_servoing.start
	end
	cur_pose = sample
	if(corridor)
	    median_curve = corridor.median_curve
	    curve_pos = median_curve.find_one_closest_point(cur_pose.position, median_curve.start_param, 0.01)
	    geom_res = (median_curve.end_param - median_curve.start_param) / median_curve.curve_length
	    #4 meter lock ahead
	    curve_pos = [curve_pos + geom_res * 4.0, median_curve.end_param].min
	    target_point = median_curve.get(curve_pos)
	    
	    direction = (target_point - cur_pose.position)
	    heading = Eigen::Vector3.UnitY.angle_to(direction)
	    
	    #convert global heading to odometry heading
	    heading_world = Eigen::Vector3.UnitY.angle_to(cur_pose.orientation * Eigen::Vector3.UnitY)
	    if(odo_sample)
		
		if((cur_pose.time - odo_sample.time).to_f.abs > 0.4)
		    puts("Warning, global and odometry times are diverged ")
		end
		
		heading_odometry = Eigen::Vector3.UnitY.angle_to(odo_sample.orientation * Eigen::Vector3.UnitY)

		final_heading = heading - (heading_world - heading_odometry)
	    
		if(final_heading < 0)
		    final_heading += 2* Math::PI
		end

		if(final_heading > 2* Math::PI)
		    final_heading -= 2* Math::PI
		end

		hd_writer.write(final_heading)
		
		#ignore z
		direction.z = 0
		#we are finished if we are within 20 cm to the goal
		if(direction.norm() < 0.2)
		    puts("CS: reached target position #{target_point}")
		end
	    end
	end
	sample
    end

    
    
    corridor_servoing.gridDump.connect_to do |s, name|
	puts("got grid dump")
	mapview.updateData(s)
	s
    end
    
    corridor_servoing.debugVfhTree.connect_to  do |s, name|
# 	puts("Got tree")
	treeview.updateData(s)
	s
    end
    
    corridor_servoing.vfhDebug.connect_to  do |data, name|
	data = data
	segment = []
	segment << data.horizonOrigin + data.horizonVector * 0.5
	segment << data.horizonOrigin - data.horizonVector * 0.5
	treeview.updateSegment(segment)
# 	puts("Got debug")
	data
    end
#     Vizkit.control log

#     test = true
#     while(test)
# 	log.step
# 	Vizkit.process_events
# 	if(map)
# 	    puts("Map id is #{map_id}")
# 	    corridor_servoing.setMap(map, map_id, pose)
# 	    if(cnt == 0)
# 		break
# 	    end
# 	    cnt = cnt + 1
# 	    map = nil
# 	end
#     end
#     
#     sleep(0.5)
#     
#     corridor_servoing.start

    

    

    
#     hd_writer.write(Math::PI / 2)
#    hd_writer.write(0)
    
#     gd_reader = corridor_servoing.gridDump.reader()
#     tr_reader = corridor_servoing.trajectory.reader()
#     
#     while(!gd_reader.read() && corridor_servoing.state == :RUNNING)
# 	sleep(0.001)
# 	log.step
#     end
#     puts("Grid was dumped")
# 
#     while((res = tr_reader.read()).size == 0 && corridor_servoing.state == :RUNNING)
# 	sleep(0.001)
# 	log.step
#     end
#     puts("Got trajectory")
    
    Vizkit.control log
    Vizkit.exec()
end

