#! /usr/bin/env ruby

#
# this test will take a log_dir and extract the logs relevant 
# for the state estimation (imu, gps), and feed the data
# to the state estimator. Finally the estimates states are 
# read and dumped themselves.
#

require 'orocos'
include Orocos
require 'pocolog'
include Pocolog
require 'readline'
require 'pocolog/stream_aligner'
require 'eigen'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end

Orocos.initialize

logreg = Typelib::Registry.new 

# Get the streams we are interested in
state_log = Logfiles.new File.open("#{ARGV[0]}/xsens_imu.0.log"), logreg
laser_log = Logfiles.new File.open("#{ARGV[0]}/hokuyo.0.log"), logreg
lowlevel_log = Logfiles.new File.open("#{ARGV[0]}/lowlevel.0.log"), logreg

streams = Array.new
streams[0]  = state_log.stream("xsens_imu.orientation_samples")
streams[1]  = laser_log.stream("hokuyo.scans")
streams[2]  = lowlevel_log.stream("odometry.odometry_samples")
joint = StreamAligner.new(false, *streams)

# setup the environment so that ruby can find the test deployment
ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos::Process.spawn 'corridorNavigationTest', 'valgrind'=> false, "wait" => 100 do |cn| 
#STDERR.puts Orocos.registry.to_xml
    corridor_servoing = cn.task 'corridor_servoing'
    
#     logger = p.task 'visual_servoing_Logger'
# 
#     logger.reportPort('visual_servoing', 'slopeBinDebug')
# 
#     logger.configure()
#     logger.start()

    laserwriter_cs = corridor_servoing.scan_samples.writer(:type => :buffer, :size => 1000)
    heading_writer_cs = corridor_servoing.heading.writer

    odometry_writer = corridor_servoing.odometry_samples(:type => :buffer, :size => 1000)
    
    corridor_servoing.search_horizon = 1.0
    search_conf = corridor_servoing.search_conf
    search_conf.stepDistance = 0.1
#    search_conf.maxTreeSize = 100
    search_conf.maxSeekSteps = 600
    search_conf.discountFactor = 1.0
    search_conf.identityThreshold = search_conf.stepDistance / 4;
    #this should be sin(angularSamplingMin) * stepDistance = stepDistance / 5
    search_conf.angularSamplingMin = Math::PI / 60
    search_conf.angularSamplingMax = Math::PI / 10
    search_conf.angularSamplingNominalCount = 5
    corridor_servoing.search_conf = search_conf
    
    cost_conf = corridor_servoing.cost_conf
    cost_conf.obstacleSenseRadius = 0.9
    cost_conf.oversamplingWidth = 15.0 / 180.0 * Math::PI
    cost_conf.speedProfile        = [0.6, 1.0 * (2.0 / Math::PI)] # 60cm/s on a straight line, 10cm/s to turn Math::PI/2 over a meter
    cost_conf.pointTurnThreshold  = Math::PI / 4
    cost_conf.pointTurnSpeed      = Math::PI / 4
    cost_conf.speedAfterPointTurn = 0.6
    cost_conf.baseTurnCost = 0.0
    corridor_servoing.cost_conf = cost_conf

    corridor_servoing.configure
    corridor_servoing.start
    
#    Readline::readline('Press enter for next sample')

    counter = 0;
    stream_idx = nil

    first = 0;
    
    #grep first imu sample, to get inital heading
    while(stream_idx != 0)
	step_info = joint.step
	stream_idx, time, data = step_info
    end
    
    stream_idx, time, data = step_info
    quat = data.orientation
    heading = quat.to_euler(2,1,0).x() +Math::PI / 2.0

    heading_writer_cs.write(heading)
    
    while step_info = joint.step
	if(counter > 10 && first > 440)
	    counter = 0;
	    Readline::readline('Press enter for next sample')
	end
	
	stream_idx, time, data = step_info
	sleep(0.001)
	if(stream_idx == 0)
	end
	if(stream_idx == 1)
	    counter = counter +1
	    first = first + 1
	    laserwriter_cs.write(data)	    
	end
	if(stream_idx == 2)
	    odometry_writer.write(data)
	end
	
    end
    
    Readline::readline('Press enter to exit')

	
end

