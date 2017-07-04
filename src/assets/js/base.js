// Cache selectors
var last_item,
    toc = $(".toc"),
    top_offset = 45,
    toc_items = toc.find("a.treeitem"),
    scroll_items = toc_items.map(function() {
        var item = $($(this).attr("href"));
        if (item.length) {
            return item;
        }
    });

function scrollToSection(id) {
    var offsetTop = $(id).offset().top - top_offset + 2;
    window.location.hash = id;
    $('html, body').scrollTop(offsetTop);
}

function checkSizeAndTOC() {
    if ($('.htb__tabs').css('display') == 'none') {
        $('body').addClass('mobile');
        resetContentHeight();

    } else {
        $('body').removeClass('mobile');
        minContentHeight();
    }
    updateTOC();
}

function updateTOC() {
    var from_top = $(this).scrollTop() + top_offset + 2;
    var cur = scroll_items.map(function() {
        if ($(this).offset().top < from_top)
            return this;
    });

    cur = cur.length == 0 ? scroll_items[0] : cur[cur.length - 1];
    var id = cur.attr('id');

    if (last_item !== id) {
        last_item = id;
        toc_items.removeClass("active").filter("[href='#" + id + "']").addClass("active");
    }

    $(".section-title").removeClass("active");

    $('.l1, .l2').hide();
    $('.toc').find('.treeitem').removeClass('treeparent');
    $('.treeitem.active').next('div').show();
    $('.treeitem.active').parents('.l0, .l1, .l2').show().prev('.treeitem').not('.active').addClass('treeparent');

    $(".treeitem.active").parents(".l0, .l1, .l2").find(".subsection-title").addClass("active");
    $(".treeitem.active").parents(".l0").prevAll(".section-title:first").addClass("active");

}


function resetContentHeight() {
    $('.content_wrapper').css('height', 'auto');
}

function minContentHeight() {
    var content_height = $('.content_wrapper').outerHeight();
    var min_height = $(window).outerHeight() - $('#htb__top').outerHeight();

    if(content_height < min_height) {
        $('.content_wrapper').css('height', min_height);
    }

}


// handlers
$(window).load(function() {
    $(".fancybox").fancybox({
        padding: 5,
        helpers: {
            title: {
                type: 'inside'
            }
        }
    });

    var inital_section = window.location.hash.substring(1);

    if (inital_section)
        scrollToSection("#" + inital_section);

    checkSizeAndTOC();
});

$(window).on('scroll', function() {
    updateTOC();
});

var resize_delay;

$(window).on('resize', function() {
    clearTimeout(resize_delay);
    resize_delay = setTimeout(checkSizeAndTOC, 50);
});

$(".slider").on('click', function() {
    var x = $(this).parent();

    $(this).hide();
    $(".content", x).show();
});

toc_items.on('click', function(e) {
    e.preventDefault();
    var href = $(this).attr("href");
    if (href === "#") {
        window.location.hash = "";
        $('html, body').scrollTop(0);
    } else {
        scrollToSection(href);
    }
});

$('.htb__tgl_menu').on('click', function() {
    $('.htb__2nd_menu').toggleClass('htb__hide');
});

$('.tgl_toc a').on('click', function(event) {
    event.preventDefault();
    $('.toc_wrapper, .tgl_toc').toggleClass('expanded');
});

