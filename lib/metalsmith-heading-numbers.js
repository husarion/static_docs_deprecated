var cheerio = require('cheerio');
var _ = require("lodash");

var util = require("util");
var extname = require('path').extname;

/**
 * Expose `plugin`.
 */

module.exports = plugin;

/**
 * A Metalsmith plugin to add an id + anchor to all headings on a page
 * ideal for permalinks
 * adapted from code and idea by remy sharp
 * (blog post: http://remysharp.com/2014/08/08/automatic-permalinks-for-blog-posts/)
 * (src file: https://github.com/remy/permalink/blob/master/permalink.js) !
 *
 * @param {Object} options
 * @return {Function}
 */

function plugin(options) {

    var opts = (typeof options === 'object') && options || {};

    // set default options or args
    opts.allow = opts.allow || false;
    opts.linkTemplate = opts.linkTemplate || '<a class="heading-anchor" href="#%s"><span></span></a>';
    opts.headingClass = opts.headingClass || '';
    opts.position = opts.position || 'left';

    var h1_starting=0;

    return function (files, metalsmith, done) {
        setImmediate(done);

        Object.keys(files).forEach(function (file) {


            if (!html(file)) return;

            // should we check if headingsIdentifier should be run based on file metakey?
            if (opts.allow !== false) {
                // metakey provided in options, check if it's false, abort!
                if (files[file][opts.allow] !== true) {
                    return;
                }
            }

            ++h1_starting;

            var idcache = {}; // store to handle duplicate ids
            var data = files[file];

            var title = data.title;
            var onepager = data.onepager;

            // load contents with cheerio to parse html nodes
            var $ = cheerio.load(data.contents.toString());

            // Set context if opts.context is provided
            var context = opts.context ? $(opts.context) : undefined;

            // Set selector if opts.selector is provided
            var selector = opts.selector || "h1,h2,h3,h4,h5,h6";

            var h1 = 0, h2 = 0, h3 = 0;

            $(selector, context).each(function (index, element) {

                var prefix1 = '', prefix2 = '', prefix3 = '';


                if(onepager) {
                    if ($(element).is('h1')) {
                        h1++;
                        h2 = 0;
                        h3 = 0;
                        prefix1 = '' + h1 + '.';
                    }
                } else {
                    if ($(element).is('h1')) {
                        h1=data.order;
                        h2 = 0;
                        h3 = 0;
                        if(data.order == null) {
                            prefix1 = '';
                        } else {
                            prefix1 = '' + h1 + '.';
                        }
                    }
                }



                if ($(element).is('h2')) {
                    h2++;
                    h3 = 0;
                    if(h1) {
                        prefix1 = h1 + '.';
                    }
                    prefix2 = h2 + '.';
                }

                // if ($(element).is('h3')) {
                //     h3++;
                //     if(h1) {
                //         prefix1 = h1 + '.';
                //     }
                //     prefix2 = h2 + '.';
                //     prefix3 = h3 + '.';
                // }

                title = title.replace(/&.*?;/g, '').replace(/\s+/g, '-').replace(/[^\w\-]/g, '').toLowerCase();

                // for each heading, check its id (and set if undefined) then append the anchor
                var id = $(element).attr("id");

                if($(element).is('h1')) {
                    id = title;
                } else {
                    id = title+'-'+id;

                }

                // if(id != title) {
                // }

                if (!id) {
                    // generate a unique one...
                    id = title+(($(element).text()).replace(/&.*?;/g, '').replace(/\s+/g, '-').replace(/[^\w\-]/g, '').toLowerCase());
                }

                if (_.isNumber(idcache[id])) {
                    // increment index for this id
                    idcache[id] += 1;

                    // duplicate id, add index to make it unique
                    id = id + '-' + idcache[id];
                }

                idcache[id] = 0; // remember id in store (duplicates must also be saved.)

                $(element).attr("id", id); // set the id

                var text = $(element).text();

                $(element).text(prefix1 + prefix2 + prefix3 + ' ' + text);

                // add heading classes
                $(element).addClass(opts.headingClass);

                // append link
                if (opts.position === 'left') {
                    $(element).prepend(util.format(opts.linkTemplate, id));
                } else {
                    $(element).append(util.format(opts.linkTemplate, id));
                }
            });

            data.contents = new Buffer($.html()); // fixes #4 - we always need to use a buffer
            files[file] = data;
        });
    };
}

/**
 * Check if a `file` is html.
 *
 * @param {String} file
 * @return {Boolean}
 */

function html(file) {
    return /\.html?/.test(extname(file));
}
