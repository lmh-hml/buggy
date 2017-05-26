#include <kangaroo_driver/kangaroo_driver_loop.hpp>


int main( int argc, char *argv[] )
{
	ros::NodeHandle *nh = NULL;
	ros::NodeHandle *nh_priv = NULL;
	kangaroo *kang = NULL;
	bool running=true;

	ros::init( argc, argv, "kangaroo_node" );

	nh = new ros::NodeHandle( );
	if( !nh )
	{
		ROS_FATAL( "Failed to initialize NodeHandle" );
		ros::shutdown( );
		return -1;
	}
	nh_priv = new ros::NodeHandle( "~" );
	if( !nh_priv )
	{
		ROS_FATAL( "Failed to initialize private NodeHandle" );
		delete nh;
		ros::shutdown( );
		return -2;
	}
	kang = new kangaroo( *nh, *nh_priv );
	if( !kang )
	{
		ROS_FATAL( "Failed to initialize driver" );
		delete nh_priv;
		delete nh;
		ros::shutdown( );
		return -3;
	}
	if( !kang->start( ) )
	{
		running = false;
		ROS_ERROR( "Failed to start the driver" );
	}
	while( ros::ok() )
	{	
		ros::spinOnce();
		if( running )kang->update();
		kang->reportState();
	
	}


	delete kang;
	delete nh_priv;
	delete nh;

	return 0;
}
