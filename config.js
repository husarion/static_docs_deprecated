var config = {};

config.pages = {
    'core2/tutorials/howtostart': {
        pattern: 'core2/tutorials/howtostart/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Tutorials',
            description: 'How to start'
        }
    },

    'core2/tutorials/ros-tutorials': {
        pattern: 'core2/tutorials/ros-tutorials/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Tutorials',
            description: 'ROS Tutorials'
        }
    },


    'core2/manuals/hardware': {
        pattern: 'core2/manuals/hardware/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Manuals',
            description: 'Hardware'
        }
    }
};

module.exports = config;