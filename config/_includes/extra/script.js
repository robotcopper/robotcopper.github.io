(function () {
  function setArchiveOpen(open) {
    $(".sidebar-wrap-2, .sidebar-wrap, .content-wrap, .archive").toggleClass("shift2", open);
    $("#toggle2").attr("aria-expanded", open ? "true" : "false");
    var $icon = $("#toggle2 .fa");
    $icon.toggleClass("fa-caret-left", !open);
    $icon.toggleClass("fa-caret-right", open);
  }

  function toggleArchive() {
    setArchiveOpen(!$(".archive").hasClass("shift2"));
  }

  $(document).on("click", "#toggle2", function (e) {
    e.preventDefault();
    e.stopPropagation();
    toggleArchive();
  });

  $("#toggle").click(function () {
    $(".sidebar-wrap-2").toggleClass("shiftbis");
  });
})();

(function () {
  var MOBILE_MQ = "(max-width: 767px)";

  function isMobileShell() {
    return window.matchMedia(MOBILE_MQ).matches;
  }

  function ensureBackdrop() {
    var el = document.querySelector(".mobile-nav-backdrop");
    if (!el) {
      el = document.createElement("div");
      el.className = "mobile-nav-backdrop";
      el.setAttribute("aria-hidden", "true");
      document.body.appendChild(el);
    }
    return el;
  }

  function closeMobileNav() {
    if (!isMobileShell()) return;
    $(".sidebar-wrap, .content-wrap, .addons-wrap").removeClass("shift");
    document.body.classList.remove("nav-open");
  }

  function syncMobileNav() {
    if (!isMobileShell()) {
      document.body.classList.remove("nav-open");
      return;
    }
    var open = $(".sidebar-wrap").hasClass("shift");
    document.body.classList.toggle("nav-open", open);
    $(".content-wrap, .addons-wrap").removeClass("shift");
  }

  $(function () {
    ensureBackdrop();
    $(".sidebar-wrap-2").removeClass("shiftbis");

    $("#toggle").on("click.mobileShell", function () {
      if (!isMobileShell()) return;
      window.setTimeout(syncMobileNav, 0);
    });

    $(document).on("click.mobileShell", ".mobile-nav-backdrop", closeMobileNav);

    $(document).on("keydown.mobileShell", function (e) {
      if (e.key !== "Escape") return;
      if (isMobileShell()) {
        closeMobileNav();
      } else if ($(".archive").hasClass("shift2")) {
        $(".sidebar-wrap-2, .sidebar-wrap, .content-wrap, .archive").removeClass("shift2");
        $("#toggle2").attr("aria-expanded", "false");
        $("#toggle2 .fa").removeClass("fa-caret-right").addClass("fa-caret-left");
      }
    });

    $(document).on("click.mobileShell", ".sidebar-wrap .toctree a", function () {
      if (!isMobileShell()) return;
      window.setTimeout(closeMobileNav, 50);
    });

    $(window).on("resize.mobileShell", function () {
      if (!isMobileShell()) {
        document.body.classList.remove("nav-open");
        $(".sidebar-wrap-2").removeClass("shiftbis");
      } else {
        syncMobileNav();
      }
    });
  });
})();

document.addEventListener("click", function (event) {
  if (event.target.classList.contains("lightbox")) {
    event.preventDefault();

    document.querySelectorAll("video").forEach(function (video) {
      video.pause();
      video.currentTime = 0;
    });

    window.location.hash = "";
    window.scrollTo(0, sessionStorage.getItem("scrollPosition"));
  }
});

document.querySelectorAll(".gallery a").forEach(function (anchor) {
  anchor.addEventListener("click", function () {
    sessionStorage.setItem("scrollPosition", window.pageYOffset);
  });
});
