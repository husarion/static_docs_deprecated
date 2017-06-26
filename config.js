var config = {};

config.pages = {
    'core2/tutorials/howtostart': {
        pattern: 'core2/tutorials/howtostart/*.md',
        sortBy: 'order',
        metadata: {
            description: 'How to start'
        }
    },
    'core2/tutorials/ros-tutorials': {
        pattern: 'core2/tutorials/ros-tutorials/*.md',
        sortBy: 'order',
        metadata: {
            description: 'ROS Tutorials'
        }

    }
};

module.exports = config;