var cheerio = require('cheerio');
var _ = require("lodash");

var util = require("util");
var extname = require('path').extname;


module.exports = plugin;

function plugin(options) {

    var opts = (typeof options === 'object') && options || {};

    // set default options or args

    return function (files, metalsmith, done) {
        setImmediate(done);

        Object.keys(files).forEach(function (file) {
            if (!html(file)) return;

            var data = files[file];

            // load contents with cheerio to parse html nodes
            var $ = cheerio.load(data.contents.toString());

            // Set context if opts.context is provided
            var context = opts.context ? $(opts.context) : undefined;

            var selector = ".gallery";

            var g_id = 0;

            $(selector, context).each(function (index, element) {
                g_id++;
                $(element).find('img').each(function (i, el) {

                    // var alt = $(el).prop('alt');
                    // $(el).removeAttr('title');
                    // $(el).removeAttr('alt');

                    var href = $(el).prop('src');
                    var title = $(el).prop('title');
                    var wrap_by = '<a class="fancybox" rel="gallery' + g_id + '" title="' + title + '" href="'+href+'"></a>';
                    $(el).wrap(wrap_by);
                });
            });

            $('.fancybox').each(function(i, el) {
                var desc = $(el).find('img').prop('alt');
                $(el).find('img').after('<span>'+desc+'</span>');
            });

            data.contents = new Buffer($.html()); // fixes #4 - we always need to use a buffer
            files[file] = data;
        });
    };
}


function html(file) {
    return /\.html?/.test(extname(file));
}
