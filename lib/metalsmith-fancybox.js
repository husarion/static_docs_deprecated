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
            var selector = ".thumb";

            $(selector, context).each(function (index, element) {

                var img = $(element).find('img');
                var href = img.prop('src');
                var img_title = img.prop('title');

                if (img_title === undefined) {
                    title = ' ';
                } else {
                    title=' title="'+img_title+'" ';
                }

                var wrap_by = '<a class="fancybox single"' + title + 'href="'+href+'"></a>';
                $(element).wrap(wrap_by);
            });


            $('.fancybox.single').each(function(i, el) {
                var desc = $(el).find('img').prop('alt');

                if(desc !== '') {
                    $(el).find('img').after('<span>'+desc+'</span>');
                }

            });

            data.contents = new Buffer($.html()); // fixes #4 - we always need to use a buffer
            files[file] = data;
        });
    };
}


function html(file) {
    return /\.html?/.test(extname(file));
}
