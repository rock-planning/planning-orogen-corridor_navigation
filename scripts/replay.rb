#! /usr/bin/env ruby

#
# this test will take a log_dir and extract the logs relevant 
# for the state estimation (imu, gps), and feed the data
# to the state estimator. Finally the estimates states are 
# read and dumped themselves.
#

require 'orocos'
include Orocos
require 'simlog'
include Pocosim
require 'readline'
require 'simlog/stream_aligner'
require 'eigen'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end

Orocos.initialize

logreg = Typelib::Registry.new 

# Get the streams we are interested in
state_log = Logfiles.new File.open("#{ARGV[0]}/xsens_imu.0.log"), Orocos.registry
laser_log = Logfiles.new File.open("#{ARGV[0]}/hokuyo.0.log"), logreg
hbridge_log = Logfiles.new File.open("#{ARGV[0]}/lowlevel.0.log"), logreg

streams = Array.new
streams[0]  = state_log.stream("xsens_imu.orientation_samples")
streams[1]  = laser_log.stream("hokuyo.scans")
streams[2]  = hbridge_log.stream("hbridge.status")
joint = StreamAligner.new(false, *streams)

# setup the environment so that ruby can find the test deployment
ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos::Process.spawn 'corridorTest', 'lowlevel', 'valgrind'=> false, "wait" => 100 do |cs, lowlevel| 
#STDERR.puts Orocos.registry.to_xml
    corridor_servoing = cs.task 'corridor_servoing'
    odometry = lowlevel.task 'odometry'
    
#     logger = p.task 'visual_servoing_Logger'
# 
#     logger.reportPort('visual_servoing', 'slopeBinDebug')
# 
#     logger.configure()
#     logger.start()

    hbdata = Orocos.registry.get('hbridge/Status').new
    
    laserwriter_cs = corridor_servoing.scan_samples.writer(:type => :buffer, :size => 10)
    heading_writer_cs = corridor_servoing.heading.writer
    orientation_writer_od = odometry.orientation_samples.writer(:type => :buffer, :size => 10)
    hbridge_writer_od = odometry.hbridge_samples.writer(:type => :buffer, :size => 10)
    
    odometry.odometry_samples.connect_to(corridor_servoing.odometry_samples, :type => :buffer, :size => 50)
    
    corridor_servoing.configure
    corridor_servoing.start
    
    odometry.configure
    odometry.start
    
#    Readline::readline('Press enter for next sample')

    counter = 0;
    stream_idx = nil

    #grep first imu sample, to get inital heading
    while(stream_idx != 0)
	step_info = joint.step
	stream_idx, time, data = step_info
    end
    
    stream_idx, time, data = step_info
    quat = Eigen::Quaternion.new(data.orientation.re, data.orientation.im[0], data.orientation.im[1], data.orientation.im[2])
    heading = quat.to_euler(2,1,0).x()

    heading_writer_cs.write(heading)
    
    while step_info = joint.step
	if(counter > 100)
	    counter = 0;
	    Readline::readline('Press enter for next sample')
	end
	
	stream_idx, time, data = step_info
	#sleep(0.01)
	if(stream_idx == 0)
	    orientation_writer_od.write(data)
	end
	if(stream_idx == 1)
	    counter = counter +1;
	    laserwriter_cs.write(data)	    
	end
	if(stream_idx == 2)
	    hbdata.time.seconds = data.time.seconds
	    hbdata.time.microseconds = data.time.microseconds 
	    (0..3).each  do|i|
		hbdata.states[i].current = data.states[i].current
		hbdata.states[i].position = data.states[i].position
		hbdata.states[i].positionExtern = data.states[i].positionExtern
		hbdata.states[i].pwm = data.states[i].pwm
	    end
	    hbridge_writer_od.write(hbdata)
	end
	
    end
    
    Readline::readline('Press enter to exit')

	
end

