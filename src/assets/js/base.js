// Cache selectors

$('.toc').find('a.treeitem').each(function() {
    var href = $(this).attr("href");

    if (href.match("^\/")) {
        href = $(this).attr('href');
        str = href.substring(href.indexOf("#"));

        if(window.location.pathname == href+'/') {
            $(this).addClass('current-page');
        }

        if(window.location.pathname == href.substring(0, href.indexOf("#"))+'/') { // wywalamy domene
            $(this).attr('href', str);
        } else {
            $(this).removeClass('treeitem').addClass('subsection-title');
        }
    }
});


var last_item,
    toc = $(".toc"),
    top_offset = 47,
    toc_items = toc.find("a.treeitem");

scroll_items = toc_items.map(function() {
    var href = $(this).attr("href");
    var item = $(href);
    if (item.length) {
        return item;
    }
});

function scrollToSection(id) {
    var offsetTop = $(id).offset().top - top_offset;
    window.location.hash = id;
    $('html, body').scrollTop(offsetTop);
}

function checkSize() {
    if ($('.htb__tabs').css('display') == 'none') {
        $('body').addClass('mobile');
        resetContentHeight();
    } else {
        $('body').removeClass('mobile');
        minContentHeight();
    }
}

function updateTOC() {
    var from_top = $(this).scrollTop() + top_offset + 10;

    var cur = scroll_items.map(function() {
        var el = $(this);
        if (el.offset().top < from_top)
            return this;
    });

    cur = cur.length == 0 ? scroll_items[0] : cur[cur.length - 1];

    var id = cur.attr('id');

    if (last_item !== id || (! ($('.active').lenght))) {
        last_item = id;
        toc_items.removeClass("active").filter("[href='#" + id + "']").addClass("active");
    }

    if(history.pushState) {
        window.history.pushState(null, null, '#' + id);
    } else {
        window.location.hash = id;
    }

    $(".section-title, .subsection-title").removeClass("active");

    $('.l1, .l2').hide();
    $('.toc').find('.treeitem').removeClass('treeparent');

    var active_item = $('.treeitem.active');

    active_item.next('div').show();
    active_item.parents('.l0, .l1, .l2').show().prev('.treeitem').not('.active').addClass('treeparent');
    active_item.parents(".l0, .l1, .l2").find(".subsection-title:first").addClass("active");
    active_item.parents(".l0").prevAll(".section-title:first").addClass("active");

}


function resetContentHeight() {
    $('.content_wrapper').css('height', 'auto');
}

function isMainPage() {
    if($(this).scrollTop() < top_offset) {
        $('.toc .active').removeClass('active');
        $('.current-page').addClass('active treeitem');
        $('.current-page.active').parents('.section').find('.section-title').addClass('active');

        if(history.pushState) {
            window.history.pushState(null, null, window.location.href.split('#')[0]);
        } else {
            window.location.href.split('#')[0];
        }
        return true;

    } else {
        $('.current-page').removeClass('treeitem');
        return false;
    }

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


    checkSize();

    var inital_section = window.location.hash.substring(1);

    if (inital_section) {
        scrollToSection("#" + inital_section);
    } else {
        isMainPage();
    }

});

var resize_delay;

$(window).on('scroll', function() {
    checkSize();

    if (!(isMainPage())) {
        updateTOC();
    }

}).on('resize', function() {
    clearTimeout(resize_delay);
    resize_delay = setTimeout(checkSize, 50);
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


$(".slider").on('click', function() {
    var x = $(this).parent();
    $(this).hide();
    $(".content", x).show();
});

$(document).ready(function() {
   $("#main-menu .dropdown-content a").on('click', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .services-dropdown-toggle").on('mouseenter', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
       $("#main-menu .docs-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .product-dropdown-toggle").on('mouseenter', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "block");
       $("#main-menu .docs-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .docs-dropdown-toggle").on('mouseenter', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
       $("#main-menu .docs-dropdown-menu").css("display", "block");
   }));
   $("#main-menu .dropdown-content").on('mouseleave', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
       $("#main-menu .docs-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .community").on('mouseenter', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
       $("#main-menu .docs-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .store").on('mouseenter', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
       $("#main-menu .docs-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .sign_up").on('mouseenter', (function() {
       $("#main-menu .product-dropdown-menu").css("display", "none");
       $("#main-menu .docs-dropdown-menu").css("display", "none");
   }));
   $("#main-menu .dropdown-content a.ros").on('click', (function() {
       $("#products .product.core2 .versions a.show_rpi").click();
   }));
   $("#main-menu .dropdown-content a.esp").on('click', (function() {
       $("#products .product.core2 .versions a.show_esp").click();
   }));
});
