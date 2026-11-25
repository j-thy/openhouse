// lh::ParseVersion / VersionCommit / CompareBuilds (include/lh/version.hpp): the update checks
// and the installer's update-or-reinstall. GitHub is faked here; LH_NETWORK_TESTS=1 also asks
// the real one about commits on master.
#include <gtest/gtest.h>

#include <cstdlib>
#include <map>
#include <string>

#include "lh/github.hpp"
#include "lh/version.hpp"

namespace {
using lh::BuildOrder;
using lh::CompareBuilds;

// A fake api.github.com: path -> body ("" = failure); counts the requests.
struct FakeGithub {
  std::map<std::string, std::string> replies;
  int calls = 0;
  std::string last;
  std::string operator()(const std::string& path) {
    ++calls;
    last = path;
    auto it = replies.find(path);
    return it == replies.end() ? std::string() : it->second;
  }
};
std::string Compare(const std::string& from, const std::string& to) {
  return "/repos/YusufOzmen01/openhouse/compare/" + from + "..." + to;
}
std::string Status(const std::string& s) { return "{\"url\":\"x\",\"status\": \"" + s + "\",\"ahead_by\":1}"; }

TEST(Version, ParseOrdersReleasesAfterTheirCandidates) {
  EXPECT_LT(lh::ParseVersion("1.0.3"), lh::ParseVersion("1.0.4-rc1"));
  EXPECT_LT(lh::ParseVersion("1.0.4-rc1"), lh::ParseVersion("1.0.4-rc2"));
  EXPECT_LT(lh::ParseVersion("1.0.4-rc2"), lh::ParseVersion("v1.0.4"));
  EXPECT_EQ(lh::ParseVersion("1.0.4-4d771af-dirty"), lh::ParseVersion("1.0.4"));
  EXPECT_EQ(lh::ParseVersion("1.0.4-rc1-4d771af"), lh::ParseVersion("1.0.4-rc1"));
  EXPECT_LT(lh::ParseVersion("1.0.9"), lh::ParseVersion("1.0.10"));
}

TEST(Version, CommitOfABuild) {
  EXPECT_EQ(lh::VersionCommit("1.0.4-4d771af"), "4d771af");
  EXPECT_EQ(lh::VersionCommit("1.0.4-4d771af-dirty"), "4d771af");
  EXPECT_EQ(lh::VersionCommit("1.0.4-rc1-4d771af"), "4d771af");
  EXPECT_EQ(lh::VersionCommit("1.0.4"), "");
  EXPECT_EQ(lh::VersionCommit("1.0.4-rc1"), "");
  EXPECT_EQ(lh::VersionCommit("1.0.4-local"), "");
  EXPECT_EQ(lh::VersionCommit("v1.0.4"), "");
}

TEST(Version, DifferentNumbersNeedNoNetwork) {
  FakeGithub g;
  EXPECT_EQ(CompareBuilds("1.0.3", "1.0.4-4d771af", std::ref(g)), BuildOrder::Newer);
  EXPECT_EQ(CompareBuilds("1.0.4-4d771af-dirty", "1.0.3", std::ref(g)), BuildOrder::Older);
  EXPECT_EQ(CompareBuilds("1.0.4-rc1", "1.0.4", std::ref(g)), BuildOrder::Newer);
  EXPECT_EQ(CompareBuilds("1.0.4", "1.0.4", std::ref(g)), BuildOrder::Same);
  EXPECT_EQ(CompareBuilds("1.0.4-abc1234", "1.0.4-abc1234-dirty", std::ref(g)), BuildOrder::Same);
  EXPECT_EQ(g.calls, 0);
}

TEST(Version, TwoCommitsAskGithub) {
  FakeGithub g;
  g.replies[Compare("e5d91ea", "4d771af")] = Status("ahead");
  g.replies[Compare("4d771af", "e5d91ea")] = Status("behind");
  g.replies[Compare("1111111", "2222222")] = Status("identical");
  g.replies[Compare("3333333", "4444444")] = Status("diverged");
  EXPECT_EQ(CompareBuilds("1.0.4-e5d91ea", "1.0.4-4d771af", std::ref(g)), BuildOrder::Newer);
  EXPECT_EQ(CompareBuilds("1.0.4-4d771af", "1.0.4-e5d91ea-dirty", std::ref(g)), BuildOrder::Older);
  EXPECT_EQ(CompareBuilds("1.0.4-1111111", "1.0.4-2222222", std::ref(g)), BuildOrder::Same);
  EXPECT_EQ(CompareBuilds("1.0.4-3333333", "1.0.4-4444444", std::ref(g)), BuildOrder::Unknown);
}

TEST(Version, AReleaseIsItsTag) {
  FakeGithub g;
  g.replies[Compare("v1.0.4", "4d771af")] = Status("ahead");
  g.replies[Compare("4d771af", "v1.0.4")] = Status("ahead");
  g.replies[Compare("v1.0.4-rc1", "abcdef1")] = Status("behind");
  EXPECT_EQ(CompareBuilds("1.0.4", "1.0.4-4d771af", std::ref(g)), BuildOrder::Newer);
  EXPECT_EQ(CompareBuilds("1.0.4-4d771af", "v1.0.4", std::ref(g)), BuildOrder::Newer);   // the release came after the build
  EXPECT_EQ(CompareBuilds("1.0.4-rc1", "1.0.4-rc1-abcdef1", std::ref(g)), BuildOrder::Older);
}

TEST(Version, AnythingUnclearIsUnknown) {
  FakeGithub g;   // no replies: offline, rate limited, or a commit GitHub doesn't have
  EXPECT_EQ(CompareBuilds("1.0.4-deadbee", "1.0.4-4d771af", std::ref(g)), BuildOrder::Unknown);
  g.replies[Compare("aaaaaaa", "bbbbbbb")] = "{\"message\":\"Not Found\"}";
  g.replies[Compare("ccccccc", "ddddddd")] = "<html>rate limited</html>";
  g.replies[Compare("eeeeeee", "fffffff")] = "{\"status\": ";
  g.replies[Compare("1234567", "7654321")] = Status("something-new");
  EXPECT_EQ(CompareBuilds("1.0.4-aaaaaaa", "1.0.4-bbbbbbb", std::ref(g)), BuildOrder::Unknown);
  EXPECT_EQ(CompareBuilds("1.0.4-ccccccc", "1.0.4-ddddddd", std::ref(g)), BuildOrder::Unknown);
  EXPECT_EQ(CompareBuilds("1.0.4-eeeeeee", "1.0.4-fffffff", std::ref(g)), BuildOrder::Unknown);
  EXPECT_EQ(CompareBuilds("1.0.4-1234567", "1.0.4-7654321", std::ref(g)), BuildOrder::Unknown);
}

TEST(Version, GithubGetRefusesOddPaths) {
  EXPECT_EQ(lh::GithubGet("repos/x"), "");
  EXPECT_EQ(lh::GithubGet("/repos/x'; rm -rf ~ #"), "");
  EXPECT_EQ(lh::GithubGet("/repos/a b"), "");
}

// The real API, on commits of master. Off by default (CI and offline builds).
TEST(Version, RealGithub) {
  if (!std::getenv("LH_NETWORK_TESTS")) GTEST_SKIP() << "LH_NETWORK_TESTS=1 to ask api.github.com";
  EXPECT_EQ(CompareBuilds("1.0.4-e5d91ea", "1.0.4-4d771af", lh::GithubGet), BuildOrder::Newer);
  EXPECT_EQ(CompareBuilds("1.0.4-4d771af-dirty", "1.0.4-e5d91ea", lh::GithubGet), BuildOrder::Older);
  EXPECT_EQ(CompareBuilds("1.0.3", "1.0.3-4d771af", lh::GithubGet), BuildOrder::Newer);   // v1.0.3...4d771af
  EXPECT_EQ(CompareBuilds("1.0.4-deadbee", "1.0.4-4d771af", lh::GithubGet), BuildOrder::Unknown);   // not on GitHub
}
}  // namespace
