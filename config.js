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


    'core2/manuals': {
        pattern: 'core2/manuals/*.md',
        metadata: {
            page: 'Manuals',
            title: 'Hardware'
        }
    }
};

module.exports = config;
