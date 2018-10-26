var fs = require('fs');
var Metalsmith = require('metalsmith');
var convert = require('metalsmith-convert');

var convert_opts = [];
var convert_src = 'src/assets/img/**/*';

var big_image_width = 1280;
var small_image_width = 150;
var small_image_height = 150;

var image_sizes = [150, 600];

// convert_opts.push({
//     src: convert_src + ".jpg",
//     resize: {width: small_image_width, height: small_image_height, resizeStyle: 'aspectfit'},
//     nameFormat: "%x_%b.jpg"
// });
// convert_opts.push({
//     src: convert_src + ".png",
//     resize: {width: small_image_width, height: small_image_height, resizeStyle: 'aspectfit'},
//     nameFormat: "%x_%b.png"
// });

convert_opts.push({
    src: convert_src + ".svg",
    resize: {width: small_image_width, height: small_image_height, resizeStyle: 'aspectfit'},
    target: 'png',
    nameFormat: "%x_%b.png"
});


//
// convert_opts.push({
//     src: convert_src + ".jpg",
//     resize: {width: big_image_width, resizeStyle: 'aspectfit'},
//     nameFormat: "%x_%b.jpg"
// });
// convert_opts.push({
//     src: convert_src + ".png",
//     resize: {width: big_image_width, resizeStyle: 'aspectfit'},
//     nameFormat: "%x_%b.png"
// });

convert_opts.push({
    src: convert_src + ".svg",
    resize: {width: big_image_width, resizeStyle: 'aspectfit'},
    target: 'png',
    nameFormat: "%x_%b.png"
});



Metalsmith(__dirname)
    .use(convert(convert_opts))
    .source("./src")
    .destination("./build2")
    .build(function (err) {
        if (err) {
            console.log(err);
        } else {
            console.log("Images created correctly");
        }
    });
