var config = {};

config.pages = {
    'core2/tutorials/howtostart': {
        pattern: 'core2/tutorials/howtostart/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Tutorials',
            title: 'How to start'
        }
    },

    'core2/tutorials/ros-tutorials': {
        pattern: 'core2/tutorials/ros-tutorials/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Tutorials',
            title: 'ROS Tutorials'
        }
    },


    'core2/manuals/hardware': {
        pattern: 'core2/manuals/hardware.md',
        metadata: {
            page: 'Manuals',
            title: 'Hardware'
        }
    },
    
    'core2/manuals/rosbot-manual': {
        pattern: 'core2/manuals/rosbot-manual.md',
        metadata: {
            page: 'Manuals',
            title: 'Hardware'
        }
    }
};

module.exports = config;
