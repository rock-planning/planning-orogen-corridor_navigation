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


#load log file 
log = Orocos::Log::Replay.open(ARGV[0])
Orocos::CORBA::max_message_size = 100000000
Bundles.initialize
Nameservice::enable(:Local)
Vizkit::ReaderWriterProxy.default_policy= Hash.new

Bundles.run "corridorNavigationTest", :valgrind => false, :output => nil do |p|
    corridor_servoing = Bundles.get('corridor_servoing')

    ls = nil
    log.filter.filtered_scans.connect_to( corridor_servoing.scan_samples, :type => :buffer, :size => 1000 ) do |s|
	ls = s
    end
    log.odometry.odometry_samples.connect_to( corridor_servoing.odometry_samples, :type => :buffer, :size => 1000 ) 

    Nameservice::Local.registered_tasks["odometry"] = log.odometry
    Nameservice::Local.registered_tasks["dynamixel"] = log.dynamixel
#     Nameservice::Local.registered_tasks["simulator"] = log.asguard_simulation

    Orocos.conf.apply( corridor_servoing, ['default'], true )
    #Orocos.conf.apply( eslam, ['default', 'localisation'], true )
    
#     log.odometry.odometry_samples.connect_to( corridor_servoing.dynamic_transformations, :type => :buffer, :size => 1000 ) 
#     log.asguard_simulation.lowerDynamixel2UpperDynamixel.connect_to( corridor_servoing.dynamic_transformations, :type => :buffer, :size => 1000 ) 
#     log.odometry.odometry_samples.connect_to( corridor_servoing.dynamic_transformations, :type => :buffer, :size => 1000 ) 
    
    Bundles.transformer.setup( corridor_servoing )
    
    corridor_servoing.configure
    
    map = nil
    pose = nil
    map_id = '/slam/mls'
    
    log.eslam_mapping.map_id do |s|
	map_id = s
	s
    end
    
    log.eslam_mapping.pose_samples do |sample|
	pose = sample
	sample
    end
    
    log.eslam_mapping.map do |sample|
	map = sample
	sample
    end
    
    cnt = 0

    view3d = Vizkit.vizkit3d_widget
    mapview = view3d.createPlugin('vfh_star', 'TraversabilityMapGeneratorVisualization')
    treeview = view3d.createPlugin('vfh_star', 'VFHTreeVisualization')
    view3d.show()
    
    corridor_servoing.gridDump.connect_to do |s|
	puts("got grid dump")
	mapview.updateData(s)
	s
    end
    corridor_servoing.debugVfhTree.connect_to  do |s|
	puts("Got tree")
	treeview.updateData(s)
	s
    end
    corridor_servoing.vfhDebug.connect_to  do |data|
	segment = []
	segment << data.horizonOrigin + data.horizonVector * 0.5
	segment << data.horizonOrigin - data.horizonVector * 0.5
	treeview.updateSegment(segment)
	puts("Got debug")
    end
#     Vizkit.control log

    test = true
    while(test)
	log.step
	Vizkit.process_events
	if(map)
	    puts("Map id is #{map_id}")
	    corridor_servoing.setMap(map, map_id, pose)
	    if(cnt == 0)
		break
	    end
	    cnt = cnt + 1
	    map = nil
	end
    end
    
    sleep(0.5)
    
    corridor_servoing.start

    hd_writer = corridor_servoing.heading.writer()
    
     hd_writer.write(Math::PI + 30.0 / 180 * Math::PI)
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

# 
# Orocos.initialize
# 
# logreg = Typelib::Registry.new 
# 
# # Get the streams we are interested in
# state_log = Logfiles.new File.open("#{ARGV[0]}/xsens_imu.0.log"), logreg
# laser_log = Logfiles.new File.open("#{ARGV[0]}/hokuyo.0.log"), logreg
# lowlevel_log = Logfiles.new File.open("#{ARGV[0]}/lowlevel.0.log"), logreg
# dynamixel_log = Logfiles.new File.open("#{ARGV[0]}/dynamixel.0.log"), logreg
# sysmon_log = Logfiles.new File.open("#{ARGV[0]}/sysmon.0.log"), logreg
# hb_log = Logfiles.new File.open("#{ARGV[0]}/lowlevel.0.log"), logreg
# 
# streams = Array.new
# streams[0]  = state_log.stream("xsens_imu.orientation_samples")
# streams[1]  = laser_log.stream("hokuyo.scans")
# streams[2]  = dynamixel_log.stream("dynamixel.lowerDynamixel2UpperDynamixel")
# streams[3]  = sysmon_log.stream("sysmon.system_status")
# streams[4]  = hb_log.stream("hbridge.status_motors")
# streams[5]  = lowlevel_log.stream("odometry.odometry_samples")
# joint = StreamAligner.new(false, *streams)
# 
# # setup the environment so that ruby can find the test deployment
# ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"
# 
# Orocos::Process.spawn 'corridorNavigationTest', 'lowlevel', 'valgrind'=> false, :valgrind_options => ["--max-stackframe=2520728"], "wait" => 100 do |cn, low| 
# #STDERR.puts Orocos.registry.to_xml
#     corridor_servoing = cn.task 'corridor_servoing'
# 
# 
#     odometry = low.task 'odometry'
# 
#     hb_writer_odo = odometry.hbridge_samples.writer(:type => :buffer, :size => 1000)
#     status_writer_odo = odometry.systemstate_samples.writer(:type => :buffer, :size => 1000)
#     imu_writer_odo = odometry.orientation_samples.writer(:type => :buffer, :size => 1000)
# 
#     #connect odometry
#     #odometry.odometry_samples.connect_to(corridor_servoing.odometry_samples, :type => :buffer, :size => 1000)
#     #odometry.bodystate_samples.connect_to(corridor_servoing.bodyState, :type => :buffer, :size => 1000)
#     
#     logger = low.task 'lowlevel_Logger'
#     logger.reportPort('odometry', 'odometry_samples')
#     logger.configure()
#     logger.start()
# 
#     sleep(1)
# #     logger = p.task 'visual_servoing_Logger'
# # 
# #     logger.reportPort('visual_servoing', 'slopeBinDebug')
# # 
# #     logger.configure()
# #     logger.start()
# 
#     laserwriter_cs = corridor_servoing.scan_samples.writer(:type => :buffer, :size => 1000)
#     heading_writer_cs = corridor_servoing.heading.writer
# 
#     odometry_writer = corridor_servoing.odometry_samples(:type => :buffer, :size => 1000)
#     
#     dyn_writer = corridor_servoing.dynamixel_samples(:type => :buffer, :size => 1000)
#     
#     corridor_servoing.search_horizon = 1.0
#     search_conf = corridor_servoing.search_conf
#     search_conf.stepDistance = 0.3
#     search_conf.maxTreeSize = 1000
# #    search_conf.maxSeekSteps = 1600
#     search_conf.discountFactor = 1.0
#     search_conf.identityThreshold = search_conf.stepDistance / 4
#     search_conf.robotWidth = 0.5
#     search_conf.obstacleSafetyDistance = 0.1
#     #this should be sin(angularSamplingMin) * stepDistance = stepDistance / 5
#     search_conf.angularSamplingMin = Math::PI / 60
#     search_conf.angularSamplingMax = Math::PI / 10
#     search_conf.angularSamplingNominalCount = 5
#     search_conf.maxStepSize = 0.2
#     corridor_servoing.search_conf = search_conf
#     
#     cost_conf = corridor_servoing.cost_conf
#     cost_conf.obstacleSenseRadius = 0.9
#     cost_conf.oversamplingWidth = 15.0 / 180.0 * Math::PI
#     cost_conf.speedProfile        = [0.6, (0.2 / (2.0 * Math::PI))] # 60cm/s on a straight line, - 0.6 m/s to turn Math::PI/2
#     cost_conf.pointTurnThreshold  = Math::PI / 4
#     cost_conf.pointTurnSpeed      = Math::PI / 4
#     cost_conf.speedAfterPointTurn = 0.6
#     cost_conf.baseTurnCost = 0.0
#     cost_conf.unknownSpeedPenalty = 0.0
#     cost_conf.shadowSpeedPenalty = 0.2
#     corridor_servoing.cost_conf = cost_conf
# 
#     corridor_servoing.transformer_max_latency = 0.1
#     
#     corridor_servoing.configure
#     corridor_servoing.start
#     
#     odometry.configure
#     odometry.start
# #    Readline::readline('Press enter for next sample')
# 
#     counter = 0;
#     stream_idx = nil
# 
#     first = 0;
#     
#     #grep first imu sample, to get inital heading
#     while(stream_idx != 0)
# 	step_info = joint.step
# 	stream_idx, time, data = step_info
#     end
#     
#     stream_idx, time, data = step_info
#     quat = data.orientation
#     heading = quat.to_euler(2,1,0).x() #+Math::PI / 2.0
# 
#     heading_writer_cs.write(heading)
#     sample = laserwriter_cs.new_sample
#     
#     while step_info = joint.step
# 
# 	stream_idx, time, data = step_info
# 	#sleep(0.001)
# 	if(stream_idx == 0)
# 	    imu_writer_odo.write(data)
# 	end
# 	if(stream_idx == 1)
# 	    counter = counter +1
# 	    first = first + 1
# 
# 	    if(counter == 10)
# 		Readline::readline('Press enter for next sample')
# 		puts(counter)
# 	    end
# 
# 	    if(counter % 100 == 0 && first > 35)
# 		Readline::readline("Press enter for next sample #{counter}")
# 		puts(counter)
# 	    end
# 	
# 
# 
#             sample.time = data.time
#             sample.start_angle = data.start_angle
#             sample.angular_resolution = data.angular_resolution
#             sample.speed = data.speed
#             sample.ranges = data.ranges
#             sample.minRange = data.minRange
#             sample.maxRange = data.maxRange
# #            sample.status = :VALID
#             sample.remission = data.remission
#             
# 	    laserwriter_cs.write(sample)	    
# 	end
# 	if(stream_idx == 2)
# 	    dyn_writer.write(data)
# #	    odometry_writer.write(data)
# 	end
# 	
# 	if(stream_idx == 3)
# 	    status_writer_odo.write(data)
# 	end
#  	if(stream_idx == 4)
# 	    hb_writer_odo.write(data)
# 	end
#  	if(stream_idx == 5)
# 	    odometry_writer.write(data)
# 	end
#     end
#     
#     Readline::readline('Press enter to exit')
# 
# 	
# end