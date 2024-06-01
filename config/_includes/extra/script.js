$("#toggle2").click(function () {
  $(".sidebar-wrap-2,.sidebar-wrap,.content-wrap, .archive").toggleClass("shift2");
});

$("#toggle").click(function () {
  $(".sidebar-wrap-2").toggleClass("shiftbis");
});

document.addEventListener('click', function(event) {
  if (event.target.classList.contains('lightbox')) {
      event.preventDefault();
      window.location.hash = '';
      window.scrollTo(0, sessionStorage.getItem('scrollPosition'));
  }
});

document.querySelectorAll('.gallery a').forEach(function(anchor) {
  anchor.addEventListener('click', function() {
      sessionStorage.setItem('scrollPosition', window.pageYOffset);
  });
});
