var Metalsmith = require('metalsmith');
var markdown = require('metalsmith-markdown');
var layouts = require('metalsmith-layouts');
var permalinks = require('metalsmith-permalinks');
var autotoc = require('metalsmith-autotoc');
var debug = require('metalsmith-debug');
var watch = require('metalsmith-watch');
var sass = require('metalsmith-sass');
var assets = require('metalsmith-assets');
var convert = require('metalsmith-convert');
var collections = require('metalsmith-collections');
var heading_numbers = require('./lib/metalsmith-heading-numbers');
var gallery = require('./lib/metalsmith-gallery');
var fancybox = require('./lib/metalsmith-fancybox');
var config = require('./config.js');
var handlebars = require('handlebars');


exports.metalsmith = function () {

    var metalsmith = Metalsmith(__dirname)
        .metadata({
            title: "Husarion Docs",
            description: "Husarion Docs",
            generator: "Metalsmith",
            url: "http://www.metalsmith.io/",
            base_url: '/',
            theme: 'light'
        })
        .source('./src')
        .destination('./build')
        .use(sass({
            source: './assets/scss',
            outputDir: 'assets/css/'   // This changes the output dir to "build/css/" instead of "build/scss/"
        }))
        .use(collections(config.pages))
        .use(markdown())
        .use(heading_numbers())
        .use(autotoc({
            selector: "h1, h2, h3, h4, h5, h6"
        }))
        .use(gallery())
        .use(fancybox())
        .use(permalinks())
        .use(layouts({
            engine: 'handlebars',
            partials: './layouts/partials'
        }));


    return metalsmith;
};

exports.build = function () {
    exports.metalsmith()
        .build(function (err, files) {
            if (err) {
                throw err;
            }
        });
};

exports.watch = function () {
    exports.metalsmith().use(watch({
        paths: {
            "${source}/**/*": true,
            "layouts/**/*": "**/*",
            "assets/scss/style.scss": "**/*"
        }
    })).build(function (err, files) {
            if (err) {
                throw err;
            }
        });
};


handlebars.registerHelper('ifCond', function (v1, operator, v2, options) {

    switch (operator) {
        case '==':
            return (v1 == v2) ? options.fn(this) : options.inverse(this);
        case '===':
            return (v1 === v2) ? options.fn(this) : options.inverse(this);
        case '!=':
            return (v1 != v2) ? options.fn(this) : options.inverse(this);
        case '!==':
            return (v1 !== v2) ? options.fn(this) : options.inverse(this);
        case '<':
            return (v1 < v2) ? options.fn(this) : options.inverse(this);
        case '<=':
            return (v1 <= v2) ? options.fn(this) : options.inverse(this);
        case '>':
            return (v1 > v2) ? options.fn(this) : options.inverse(this);
        case '>=':
            return (v1 >= v2) ? options.fn(this) : options.inverse(this);
        case '&&':
            return (v1 && v2) ? options.fn(this) : options.inverse(this);
        case '||':
            return (v1 || v2) ? options.fn(this) : options.inverse(this);
        default:
            return options.inverse(this);
    }
});


handlebars.registerHelper('var',function(name, value, options){
    options.data.root[name] = value;
});