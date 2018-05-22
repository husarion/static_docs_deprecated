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

    'core2/tutorials/husarnet': {
        pattern: 'core2/tutorials/husarnet/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Tutorials',
            title: 'Husarnet'
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

    'core2/tutorials/other-tutorials': {
        pattern: 'core2/tutorials/other-tutorials/*.md',
        sortBy: 'order',
        metadata: {
            page: 'Tutorials',
            title: 'ROS Tutorials'
        }
    },

    'core2/manuals/core2': {
        pattern: 'core2/manuals/core2.md',
        metadata: {
            page: 'Manuals',
            title: 'CORE2 manual'
        }
    },
    
     'core2/manuals/core2mini': {
        pattern: 'core2/manuals/core2mini.md',
        metadata: {
            page: 'Manuals',
            title: 'CORE2mini manual'
        }
    },
    
    'core2/manuals/telepresence': {
        pattern: 'core2/manuals/telepresence.md',
        metadata: {
            page: 'Manuals',
            title: 'Telepresence robot kit manual'
        }
    },
    
    'core2/manuals/rosbot-manual': {
        pattern: 'core2/manuals/rosbot-manual.md',
        metadata: {
            page: 'Manuals',
            title: 'ROSbot manual'
        }
    },
	
    'core2/manuals/husarion-add-ons': {
        pattern: 'core2/manuals/husarion-add-ons.md',
        metadata: {
            page: 'Manuals',
            title: 'Husarion add-ons'
        }
    },
	
    'core2/software/husarnet': {
        pattern: 'core2/software/husarnet.md',
        metadata: {
            page: 'Software',
            title: 'Husarnet (beta)'
        }
    }
};

module.exports = config;
