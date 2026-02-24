/* ============================================
   Scroll Reveal (Intersection Observer)
   ============================================ */
const revealObserver = new IntersectionObserver(
    (entries) => {
        entries.forEach((entry) => {
            if (entry.isIntersecting) {
                entry.target.classList.add('visible');
                revealObserver.unobserve(entry.target);
            }
        });
    },
    { threshold: 0.1, rootMargin: '0px 0px -50px 0px' }
);

document.querySelectorAll('.reveal, .reveal-stagger').forEach((el) => {
    revealObserver.observe(el);
});

/* ============================================
   Navbar — Scroll Background & Shrink
   ============================================ */
const navbar = document.getElementById('navbar');

window.addEventListener('scroll', () => {
    if (window.scrollY > 50) {
        navbar.classList.add('scrolled');
    } else {
        navbar.classList.remove('scrolled');
    }
});

/* ============================================
   Navbar — Active Link Highlighting
   ============================================ */
const sections = document.querySelectorAll('.section, .hero');
const navLinks = document.querySelectorAll('.nav-link');

const activeLinkObserver = new IntersectionObserver(
    (entries) => {
        entries.forEach((entry) => {
            if (entry.isIntersecting) {
                const id = entry.target.getAttribute('id');
                navLinks.forEach((link) => {
                    link.classList.remove('active');
                    if (link.getAttribute('href') === `#${id}`) {
                        link.classList.add('active');
                    }
                });
            }
        });
    },
    { threshold: 0.3, rootMargin: '-80px 0px -50% 0px' }
);

sections.forEach((section) => {
    activeLinkObserver.observe(section);
});

/* ============================================
   Mobile Menu Toggle
   ============================================ */
const navToggle = document.getElementById('navToggle');
const navMenu = document.getElementById('navMenu');

navToggle.addEventListener('click', () => {
    navToggle.classList.toggle('active');
    navMenu.classList.toggle('open');
    document.body.style.overflow = navMenu.classList.contains('open') ? 'hidden' : '';
});

// Close menu when clicking a link
navMenu.querySelectorAll('a').forEach((link) => {
    link.addEventListener('click', () => {
        navToggle.classList.remove('active');
        navMenu.classList.remove('open');
        document.body.style.overflow = '';
    });
});

/* ============================================
   Typing Animation
   ============================================ */
const typingElement = document.getElementById('typingText');
const phrases = [
    'I build autonomous robotic systems.',
    'I explore vision-language-action models.',
    'I develop intelligent embodied agents.',
    'I fuse sensors with Bayesian inference.',
    'I train robots with reinforcement learning.',
];
let phraseIndex = 0;
let charIndex = 0;
let isDeleting = false;
let typingTimeout;

function typeWriter() {
    const currentPhrase = phrases[phraseIndex];

    if (!isDeleting) {
        typingElement.textContent = currentPhrase.substring(0, charIndex + 1);
        charIndex++;

        if (charIndex === currentPhrase.length) {
            isDeleting = true;
            typingTimeout = setTimeout(typeWriter, 2000);
            return;
        }
        typingTimeout = setTimeout(typeWriter, 50);
    } else {
        typingElement.textContent = currentPhrase.substring(0, charIndex - 1);
        charIndex--;

        if (charIndex === 0) {
            isDeleting = false;
            phraseIndex = (phraseIndex + 1) % phrases.length;
            typingTimeout = setTimeout(typeWriter, 400);
            return;
        }
        typingTimeout = setTimeout(typeWriter, 30);
    }
}

// Start typing after hero animations complete
setTimeout(typeWriter, 1500);

/* ============================================
   Smooth Scroll for Nav Links
   ============================================ */
document.querySelectorAll('a[href^="#"]').forEach((anchor) => {
    anchor.addEventListener('click', function (e) {
        const href = this.getAttribute('href');
        if (href === '#') return;

        e.preventDefault();
        const target = document.querySelector(href);
        if (target) {
            target.scrollIntoView({ behavior: 'smooth' });
        }
    });
});

/* ============================================
   Contact Form — Basic Handler
   ============================================ */
const contactForm = document.getElementById('contactForm');

contactForm.addEventListener('submit', (e) => {
    e.preventDefault();

    const name = document.getElementById('name').value;
    const email = document.getElementById('email').value;
    const message = document.getElementById('message').value;

    // Open mailto as a fallback
    const subject = encodeURIComponent(`Portfolio Contact from ${name}`);
    const body = encodeURIComponent(`Name: ${name}\nEmail: ${email}\n\nMessage:\n${message}`);
    window.location.href = `mailto:gogulnath@example.com?subject=${subject}&body=${body}`;
});
